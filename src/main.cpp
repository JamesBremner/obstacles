#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <wex.h>
#include "cxy.h"
#include <autocell.h>
#include "cObstacle.h"
#include "cGUI.h"

void cObstacle::clear()
{
    myView = -999;
    myfrect = true;
    vN.clear(); ///< nodes to be included in path
    vL.clear(); ///< links between nodes
    vPath.clear();
    mySpanningTree.clear();
}

void cObstacle::obstacle(int x, int y)
{
    if (0 > x || x > nx - 1 || 0 > y || y >> ny - 1)
        throw std::runtime_error("Bad location");
    A->cell(x, y)->myType = 1;
}

void cObstacle::grid(int x, int y)
{
    nx = x;
    ny = y;
    A = new cell::cAutomaton<cOCell>(nx, ny);
}

void read(
    cObstacle &obs,
    const std::string &fname)
{
    obs.clear();
    std::ifstream ifs(fname);
    if (!ifs.is_open())
        throw std::runtime_error(
            "Cannot open input file");
    std::string form, calc;
    ifs >> form >> calc;
    if (form != "format")
        throw std::runtime_error(
            "Bad input format");

    if (calc.find("obs") != -1)
    {
        int nx, ny, view;
        ifs >> nx >> ny >> view;
        obs.grid(nx, ny);
        obs.view(view);
        while (ifs.good())
        {
            // insert obstacle
            int ox, oy;
            ifs >> ox >> oy;
            obs.obstacle(ox, oy);
        }
    }

    else if (calc.find("farm") != -1)
    {
        int nx, ny;
        std::string sid;
        obs.view(-1);
        ifs >> sid;
        if (sid[0] == 'p')
            obs.poly();
        else
        {
            ifs >> nx >> ny;
            obs.grid(nx, ny);
            return;
        }
        ifs >> nx >> ny;
        obs.polyAdd( cxy(nx, ny) );
        while (ifs.good()) {
            ifs >> sid >> nx >> ny;
            if( sid[0] == 'p' )
            obs.polyAdd( cxy(nx, ny) );
        }
    }
    else
        throw std::runtime_error(
            "bad calculation type ");
}

bool cObstacle::isBlocked(int x1, int y1, int x2, int y2)
{
    int W, H;
    A->size(W, H);
    for (int w = 0; w < W; w++)
        for (int h = 0; h < H; h++)
        {
            // check for obstacle
            if (A->cell(w, h)->myType != 1)
                continue;

            // check for obstacle close to connection
            int ow, oh;
            A->coords(ow, oh, A->cell(w, h));

            cxy obstacle(ow, oh);
            cxy line1(x1, y1);
            cxy line2(x2, y2);
            if (obstacle.dis2toline(line1, line2) < 2)
                return true;
        }
    // no obstacle found close to connection
    return false;
}

void cObstacle::connect()
{
    // loop over node pairs
    for (auto n1 : vN)
        for (auto n2 : vN)
        {
            // do not self connect
            if (n1 == n2)
                continue;

            // do not connect nodes that are not neighbours
            int w1, h1, w2, h2;
            A->coords(w1, h1, n1);
            A->coords(w2, h2, n2);
            int dx = w1 - w2;
            int dy = h1 - h2;
            int d2 = dx * dx + dy * dy;
            if (d2 > 50)
                continue;

            // check for blocked
            if (isBlocked(w1, h1, w2, h2))
                continue;

            // OK to connect
            vL.push_back(std::make_tuple(n1, n2, d2));
        }
}

double cObstacle::linkCost(link_t &l)
{
    int w, h;
    A->coords(w, h, std::get<0>(l));
    cxy n1(w, h);
    A->coords(w, h, std::get<1>(l));
    cxy n2(w, h);
    std::get<2>(l) = n1.dist2(n2);
    return std::get<2>(l);
}

void cObstacle::unobstructedPoints()
{
    int W, H;

    if( myfrect )
        A->size(W, H);
    else {
        cxy wh = cxy::enclosingWidthHeight( myPolygon );
        W = wh.x;
        H = wh.y;
        grid( W, H );
    }

    int V;
    if (myView > 0)
        V = myView;
    else
        V = 2;

    for (int h = V; h < H - V + 1; h += 2 * V + 1)
        for (int w = V; w <= W - V + 1; w += 2 * V + 1)
        {
            if( ! myfrect ) {
                cxy p( w, h );
                if( ! p.isInside( myPolygon ))
                    continue;
            }
            cOCell *c = A->cell(w, h);
            c->myType = 2;
            vN.push_back(c);
        }
}

std::string cObstacle::draw(int w, int h) const
{
    switch (A->cell(w, h)->myType)
    {
    case 0:
    default:
        return "";
    case 1:
        return "X";
    case 2:
        return std::to_string(A->cell(w, h)->ID());
    }
}
std::vector<cOCell *> cObstacle::adjacent(
    cOCell *n,
    const vlink_t &vLink)
{
    std::vector<cOCell *> ret;
    for (auto &l : vLink)
    {
        if (std::get<0>(l) == n)
            ret.push_back(std::get<1>(l));
        if (std::get<1>(l) == n)
            ret.push_back(std::get<0>(l));
    }
    return ret;
}

cOCell *cObstacle::closestUnvisitedConnected(
    cOCell *v, vlink_t &vLink)
{
    double bestDist = INT_MAX;
    cOCell *ret = 0;
    cOCell *w;
    for (auto &l : vLink)
    {
        w = 0;
        if (std::get<0>(l) == v)
            w = std::get<1>(l);
        if (std::get<1>(l) == v)
            w = std::get<0>(l);
        if (!w)
            continue;
        if (w->fvisited)
            continue;
        if (linkCost(l) < bestDist)
        {
            bestDist = std::get<2>(l);
            ret = w;
        }
    }
    return ret;
}
void cObstacle::spanningTree()
{
    // add initial arbitrary link
    auto v = vN[0];
    auto w = *adjacent(v, vL).begin();
    mySpanningTree.push_back(std::make_tuple(v, w, 1));
    v->fvisited = true;
    w->fvisited = true;

    // while nodes remain outside of span
    while (1)
    {
        // int v;      // node in span
        // int w = -1; // node not yet in span
        int min_cost = INT_MAX;
        link_t bestLink;

        // loop over nodes in span
        for (int kv = 0; kv < vN.size(); kv++)
        {
            if (!vN[kv]->fvisited)
                continue;
            v = vN[kv];

            // loop over nodes not in span
            for (int kw = 0; kw < vN.size(); kw++)
            {
                if (vN[kw]->fvisited)
                    continue;
                w = vN[kw];

                // check for allowed connection
                for (auto &l : vL)
                {
                    if ((std::get<0>(l) == v && std::get<1>(l) == w) || (std::get<0>(l) == w && std::get<1>(l) == v))
                    {
                        // check for cheapest
                        if (linkCost(l) < min_cost)
                        {
                            min_cost = linkCost(l);
                            bestLink = l;
                        }
                        break;
                    }
                }
            }
        }
        if (min_cost == INT_MAX)
            break;
        mySpanningTree.push_back(bestLink);
        std::get<0>(bestLink)->fvisited = true;
        std::get<1>(bestLink)->fvisited = true;
    }
}




void cGUI::ConstructMenu()
{
    wex::menubar mbar(fm);
    wex::menu mfile(fm);
    mfile.append("Calculate",
                 [&](const std::string &title)
                 {
                     wex::filebox fb(fm);

                     try
                     {
                         read(myObstacle, fb.open());
                         myObstacle.unobstructedPoints();
                         myObstacle.connect();
                         myObstacle.spanningTree();
                         myObstacle.tourSpanningTree();
                         fm.update();
                     }
                     catch (std::runtime_error &e)
                     {
                         wex::msgbox m(e.what());
                         return;
                     }
                 });

    mbar.append("File", mfile);

    wex::menu vfile(fm);
    vfile.append("Spanning Tree",
                 [&](const std::string &title)
                 {
                     myViewType = eView::span;
                     fm.update();
                 });
    vfile.append("Route",
                 [&](const std::string &title)
                 {
                     myViewType = eView::route;
                     fm.update();
                 });
    mbar.append("View", vfile);
}

main()
{
    cGUI theGUI;
    return 0;
}
