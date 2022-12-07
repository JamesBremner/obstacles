#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <wex.h>
#include <autocell.h>
#include "cObstacle.h"
#include "cGUI.h"

/// @brief 2D point
class cxy
{

public:
    double x;
    double y;

    cxy()
    {
    }
    cxy(int X, int Y)
        : x(X), y(Y)
    {
    }
    cxy vect(const cxy &other) const;
    double dist2(const cxy &other) const;

    double dis2toline(
        const cxy &line1,
        const cxy &line2) const;
};

cxy cxy::vect(const cxy &other) const
{
    cxy v;
    v.x = other.x - x;
    v.y = other.y - y;
    return v;
}

double cxy::dist2(const cxy &other) const
{
    cxy v = vect(other);
    return v.x * v.x + v.y * v.y;
}

double cxy::dis2toline(
    const cxy &line1,
    const cxy &line2) const
{
    cxy AB = line1.vect(line2);
    cxy AP = line1.vect(*this);
    double lAB = AB.x * AB.x + AB.y * AB.y;
    double t = (AB.x * AP.x + AB.y * AP.y) / lAB;
    if (t < 0)
        t = 0;
    if (t > 1)
        t = 1;
    cxy closest;
    closest.x = line1.x + t * AB.x;
    closest.y = line1.y + t * AB.y;
    return dist2(closest);
}

void cObstacle::clear()
{
    myView = -999;
     vN.clear();    ///< nodes to be included in path
     vL.clear();                  ///< links between nodes
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
        ifs >> nx >> ny;
        obs.grid(nx, ny);
        obs.view(-1);
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
    A->size(W, H);

    int V;
    if (myView > 0)
        V = myView;
    else
        V = 2;

    for (int h = V; h < H - V + 1; h += 2 * V + 1)
        for (int w = V; w <= W - V + 1; w += 2 * V + 1)
        {
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

void cObstacle::tourSpanningTree()
{
    auto connectedLeaves = mySpanningTree;
    std::vector<cOCell *> leaves;
    for (auto v : vN)
    {
        int countSpanningTreeConnections = 0;
        for (auto &l : mySpanningTree)
        {
            if (std::get<0>(l) == v || std::get<1>(l) == v)
                countSpanningTreeConnections++;
        }
        if (countSpanningTreeConnections == 1)
            leaves.push_back(v);
    }
    for (int kv = 0; kv < leaves.size(); kv++)
        for (int kw = 0; kw < leaves.size(); kw++)
        {
            // no self cycles
            if (kv == kw)
                continue;

            // check for unblocked connection
            auto va = adjacent(leaves[kv], vL);
            if (std::find(va.begin(), va.end(), leaves[kw]) == va.end())
                continue;

            link_t l =
                std::make_tuple(
                    leaves[kv],
                    leaves[kw],
                    0);
            linkCost(l);
            connectedLeaves.push_back(l);
        }
    tour(connectedLeaves);
}

void cObstacle::tour(vlink_t &connectedLeaves)
{
    vPath.clear();
    for (auto n : vN)
        n->fvisited = false;

    link_t best_link;

    auto v = std::get<0>(vL[0]);
    std::cout << v->ID() << " ";
    v->fvisited = true;

    while (1)
    {
        while (1)
        {
            while (1)
            {
                // travel along spanning tree
                while (1)
                {
                    auto w = closestUnvisitedConnected(v, mySpanningTree);
                    if (!w)
                        break;

                    pathAdd(v, w);
                    v = w;
                }

                // travel from leaf to an unvisited connected leaf
                auto w = closestUnvisitedConnected(v, connectedLeaves);
                if (!w)
                    break;

                pathAdd(v, w);
                v = w;
            }

            // jump to closest unvisited node in spanning tree
            std::vector<cOCell *> jump_path;
            auto w = ClosestUnvisited(v, vL, jump_path);
            if (!w)
            {
                // all nodes have been visited
                return;
            }
            std::cout << " ( ";
            for (auto p : jump_path)
            {
                pathAdd(v, p);
                v = p;
            }
            std::cout << " ) ";
            v = w;
        }
    }
}

void cObstacle::pathAdd(
    cOCell *node1,
    cOCell *node2)
{
    vPath.push_back(std::make_tuple(node1, node2, 0));
    node2->fvisited = true;
    std::cout << node2->ID() << " ";
}

cOCell *cObstacle::ClosestUnvisited(
    cOCell *startp,
    vlink_t &vlink,
    std::vector<cOCell *> &path)
{
    // shortest distance from start to each node
    std::vector<double> dist(vN.size(), INT_MAX);

    // previous node on shortest path to eachj node
    std::vector<int> pred(vN.size(), -1);

    std::vector<bool> sptSet(vN.size(), false); // sptSet[i] will be true if vertex i is included in shortest
    // path tree or shortest distance from src to i is finalized

    int startIdx = std::distance(
        vN.begin(),
        std::find(vN.begin(), vN.end(),
                  startp));

    // Distance of source vertex from itself is always 0
    dist[startIdx] = 0;
    pred[startIdx] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < vN.size() - 1; count++)
    {
        // Pick the minimum distance vertex from the set of vertices not
        // yet processed. u is always equal to src in the first iteration.
        int min = INT_MAX, uidx;
        for (int vidx = 0; vidx < vN.size(); vidx++)
            if (sptSet[vidx] == false && dist[vidx] <= min)
            {
                min = dist[vidx];
                uidx = vidx;
            }
        if (min == INT_MAX)
        {
            // no more nodes connected to start
            break;
        }

        // Mark the picked vertex as processed
        sptSet[uidx] = true;

        // Update dist value of the adjacent vertices of the picked vertex.
        for (auto vp : adjacent(vN[uidx], vL))
        {
            int vidx = std::distance(
                vN.begin(),
                std::find(vN.begin(), vN.end(),
                          vp));
            if (sptSet[vidx])
                continue; // already processed

            // Update dist[v] only if total weight of path from src to  v through u is
            // smaller than current value of dist[v]
            link_t l = std::make_tuple(vN[uidx], vN[vidx], 0);
            double cost = linkCost(l);
            if (dist[uidx] + cost < dist[vidx])
            {
                dist[vidx] = dist[uidx] + cost;
                pred[vidx] = uidx;
            }
        }
    }
    double bestDist = INT_MAX;
    int best_index;
    cOCell *best = 0;
    for (int i = 0; i < dist.size(); i++)
    {
        if (vN[i]->fvisited)
            continue;
        if (dist[i] < bestDist)
        {
            best = vN[i];
            bestDist = dist[i];
            best_index = i;
        }
    }
    if( bestDist == INT_MAX )
        return best;

    path.push_back(best);
    int i = best_index;
    while (1)
    {
        i = pred[i];
        path.push_back(vN[i]);
        if (i = startIdx)
            break;
    }
    std::reverse(path.begin(), path.end());
    return best;
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
