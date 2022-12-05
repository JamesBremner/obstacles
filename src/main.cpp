#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <wex.h>
#include "cStarterGUI.h"
#include <autocell.h>

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

/// @brief 2D grid cell that can contain obstacles

class cOCell : public cell::cCell
{
public:
    cOCell()
        : cell::cCell(),
          myType(0),
          fvisited(false)
    {
    }
    /** contents type
     * 0 empty
     * 1 obstacle
     * 2 node that must be visited
     */
    int myType;

    bool fvisited;
};

typedef std::tuple<cOCell *, cOCell *, double> link_t;
typedef std::vector<link_t> vlink_t;

class cObstacle
{

    int nx, ny;                  ///< grid size
    int view;                    ///< view radius
    cell::cAutomaton<cOCell> *A; ///< 2D grid
    std::vector<cOCell *> vN;    ///< nodes to be included in path
    vlink_t vL;                  ///< links between nodes
    vlink_t vPath;
    vlink_t mySpanningTree;

public:
    /// @brief read layout of obstacles from file
    /// @param fname
    void read(const std::string &fname);

    /// @brief construct nodes to be visited
    void unobstructedPoints();

    /// @brief connect nodes, avoiding obstacles
    void connect();

    /// @brief output nodes and links
    void output();

    /// @brief display for point at w,h
    /// @param w
    /// @param h
    /// @return display
    std::string draw(int w, int h) const;

    /// @brief get grid dimensions
    /// @param w
    /// @param h
    void size(int &w, int &h) const
    {
        w = nx;
        h = ny;
    }

    /// @brief get links
    /// @return
    vlink_t links()
    {
        return vL;
    }

    /// @brief get grid
    /// @return
    cell::cAutomaton<cOCell> *grid()
    {
        return A;
    }

    /// @brief Is link blocked by obstacle
    /// @param x1,y1 col,row indeices of
    /// @param y1
    /// @param x2
    /// @param y2
    /// @return
    bool isBlocked(int x1, int y1, int x2, int y2);

    void readPath();

    vlink_t path()
    {
        return vPath;
    }
    /// @brief Find tree that connects all required nodes
    void spanningTree();

    vlink_t spanningTree_get()
    {
        return mySpanningTree;
    }

    /// @brief Find practical tour visiting all required nodes
    void tourSpanningTree();

private:
    /// @brief calculate and set link cost squared
    /// @param l
    /// @return
    double linkCost(link_t &l);

    /// @brief adjacent cells, reachable with 1 hop
    /// @param n cell
    /// @param vlink allowed links
    /// @return vector of called adjacent to n
    std::vector<cOCell *> adjacent(
        cOCell *n,
        const vlink_t &vlink);

    cOCell *closestUnvisitedConnected(
        cOCell *v, vlink_t &vLink);

    void tour(vlink_t &connectedLeaves);

    /// @brief Ffind closest unvisited node
    /// @param start start node
    /// @param vlink allowed links
    /// @param path path from start to nearest
    /// @return pointer to nearest unvisited node
    ///
    /// Uses Dijsktra alorithm
    cOCell *Dijsktra(
        cOCell *start,
        vlink_t &vlink,
        std::vector<cOCell *> &path);
};

void cObstacle::readPath()
{
    // std::ifstream ifs("path.txt");
    // if (!ifs.is_open())
    //     exit(1);
    // int id1, id2;
    // std::string arrow;
    // while (ifs.good())
    // {
    //     ifs >> id1 >> arrow >> id2;
    //     vPath.push_back(
    //         std::make_pair(
    //             A->cell(id1), A->cell(id2)));
    // }

    // std::vector<int> pid = {
    //     47, 52, 57, 157, 257, 357, 152, 252, 147, 247, 347, 352, 42, 142, 242, 342};

    // for (int k = 1; k < pid.size(); k++)
    //     vPath.push_back(
    //         std::make_pair(
    //             A->cell(pid[k - 1]), A->cell(pid[k])));
}

void cObstacle::read(const std::string &fname)
{
    std::ifstream ifs(fname);
    if (!ifs.is_open())
        exit(1);
    ifs >> nx >> ny >> view;
    A = new cell::cAutomaton<cOCell>(nx, ny);
    while (ifs.good())
    {
        // insert obstacle
        int ox, oy;
        ifs >> ox >> oy;
        A->cell(ox, oy)->myType = 1;
    }
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

// void cObstacle::output()
// {
//     for (auto n : vN)
//     {
//         std::cout << "n " << n->ID() << "\n";
//     }
//     for (auto l : vL)
//     {
//         if (std::get<0>(l)->ID() > std::get<1>(l)->ID())
//             continue;
//         // int w, h;
//         // A->coords(w, h, l.first);
//         // cxy n1(w, h);
//         // A->coords(w, h, l.second);
//         // cxy n2(w, h);

//         std::cout << "l " << std::get<0>(l)->ID()
//                   << " " << std::get<1>(l)->ID()
//                   << " " << std::get<2>(l)
//                   << "\n";
//     }
//     for (int kn1 = 0; kn1 < vN.size(); kn1++)
//         for (int kn2 = kn1 + 1; kn2 < vN.size(); kn2++)
//         {
//             bool found = false;
//             for (auto &l : vL)
//                 if ((l.first == vN[kn1] && l.second == vN[kn2]) || (l.first == vN[kn2] && l.second == vN[kn1]))
//                 {
//                     found = true;
//                     break;
//                 }
//             if (found)
//                 continue;

//             int w1, h1, w2, h2;
//             A->coords(w1, h1, vN[kn1]);
//             A->coords(w2, h2, vN[kn2]);
//             if (isBlocked(w1, h1, w2, h2))
//                 continue;

//             // missing link

//             std::cout << "l " << vN[kn1]->ID()
//                       << " " << vN[kn2]->ID()
//                       << " " << nx * ny
//                       << "\n";
//         }
// }

void cObstacle::unobstructedPoints()
{
    int W, H;
    A->size(W, H);
    for (int h = view; h < H - view + 1; h += 2 * view + 1)
        for (int w = view; w <= W - view + 1; w += 2 * view + 1)
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
                auto w = closestUnvisitedConnected(v, mySpanningTree);
                if (!w)
                    break;

                vPath.push_back(std::make_tuple(v, w, 0));
                w->fvisited = true;
                std::cout << w->ID() << " ";
                v = w;
            }

            auto w = closestUnvisitedConnected(v, connectedLeaves);
            if (!w)
                break;

            vPath.push_back(std::make_tuple(v, w, 0));
            w->fvisited = true;
            std::cout << w->ID() << " ";
            v = w;
        }

        std::vector<cOCell *> jump_path;
        auto w = Dijsktra(v, vL, jump_path);
        if (!w)
            break;

        for (auto p : jump_path)
        {
            vPath.push_back(std::make_tuple(v, p, 0));
            std::cout << p->ID() << " ";
            v = p;
        }

        v = w;

        while (1)
        {
            auto w = closestUnvisitedConnected(v, mySpanningTree);
            if (!w)
                break;

            vPath.push_back(std::make_tuple(v, w, 0));
            w->fvisited = true;
            std::cout << w->ID() << " ";
            v = w;
        }
        break;
    }
}

cOCell *cObstacle::Dijsktra(
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

class cGUI : public cStarterGUI
{
public:
    cGUI()
        : cStarterGUI(
              "Starter",
              {50, 50, 1000, 500}),
          lb(wex::maker::make<wex::label>(fm))
    {
        myObstacle.read("data.txt");
        myObstacle.unobstructedPoints();
        myObstacle.connect();
        myObstacle.spanningTree();
        myObstacle.tourSpanningTree();

        fm.events().draw(
            [&](PAINTSTRUCT &ps)
            {
                wex::shapes S(ps);
                int W, H;
                myObstacle.size(W, H);
                for (int h = 0; h < H; h++)
                    for (int w = 0; w < W; w++)
                        if (!myObstacle.draw(w, h).empty())
                            S.text(
                                myObstacle.draw(w, h),
                                {w * 20, h * 20});
                auto grid = myObstacle.grid();
                // for (auto &l : myObstacle.links())
                // {
                //     int w, h, w2, h2;
                //     grid->coords(
                //         w, h, std::get<0>(l));
                //     // std::cout << w << " " << h << " -> ";
                //     grid->coords(
                //         w2, h2, std::get<1>(l));
                //     // std::cout << w2 << " " << h2 << "\n";
                //     S.line({20 * w, 20 * h, 20 * w2, 20 * h2});
                // }
                // S.color(0x0000FF);
                // S.penThick(2);
                // for (auto &pl : myObstacle.spanningTree_get())
                // {
                //     int w, h, w2, h2;
                //     grid->coords(
                //         w, h, std::get<0>(pl));
                //     grid->coords(
                //         w2, h2, std::get<1>(pl));
                //     S.line({20 * w, 20 * h, 20 * w2, 20 * h2});
                // }
                S.color(0xFF0000);
                S.penThick(1);
                std::stringstream sspath;
                sspath << std::get<0>(myObstacle.path()[0])->ID();
                for (auto &pl : myObstacle.path())
                {
                    auto n1 = std::get<0>(pl);
                    auto n2 = std::get<1>(pl);
                    int w, h, w2, h2;
                    grid->coords(
                        w, h, n1);
                    grid->coords(
                        w2, h2, n2);
                    S.line({20 * w, 20 * h, 20 * w2, 20 * h2});
                    sspath << " -> " << n2->ID();
                }
                S.text(sspath.str(), {20, H * 20});
            });

        show();
        run();
    }

private:
    wex::label &lb;
    cObstacle myObstacle;
};

main()
{
    cGUI theGUI;
    return 0;
}
