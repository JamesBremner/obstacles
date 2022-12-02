#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
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

class cObstacle
{
    int nx, ny;                                    ///< grid size
    int view;                                      ///< view radius
    cell::cAutomaton<cOCell> *A;                   ///< 2D grid
    std::vector<cOCell *> vN;                      ///< nodes to be included in path
    std::vector<std::pair<cOCell *, cOCell *>> vL; ///< links between nodes
    std::vector<std::pair<cOCell *, cOCell *>> vPath;
    std::vector<std::pair<cOCell *, cOCell *>> mySpanningTree;

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
    std::vector<std::pair<cOCell *, cOCell *>> links()
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
    std::vector<std::pair<cOCell *, cOCell *>> path()
    {
        return vPath;
    }
    std::vector<std::pair<cOCell *, cOCell *>> spanningTree_get()
    {
        return mySpanningTree;
    }
    void spanningTree();
    std::vector<cOCell *> adjacent(cOCell *n);

private:
    double linkCost(const std::pair<cOCell *, cOCell *> &l);
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

    std::vector<int> pid = {
        47, 52, 57, 157, 257, 357, 152, 252, 147, 247, 347, 352, 42, 142, 242, 342};

    for (int k = 1; k < pid.size(); k++)
        vPath.push_back(
            std::make_pair(
                A->cell(pid[k - 1]), A->cell(pid[k])));
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
            vL.push_back(std::make_pair(n1, n2));
        }
}

double cObstacle::linkCost(const std::pair<cOCell *, cOCell *> &l)
{
    int w, h;
    A->coords(w, h, l.first);
    cxy n1(w, h);
    A->coords(w, h, l.second);
    cxy n2(w, h);
    double dbg = n1.dist2(n2);
    return sqrt(n1.dist2(n2));
}
void cObstacle::output()
{
    for (auto n : vN)
    {
        std::cout << "n " << n->ID() << "\n";
    }
    for (auto l : vL)
    {
        if (l.first->ID() > l.second->ID())
            continue;
        int w, h;
        A->coords(w, h, l.first);
        cxy n1(w, h);
        A->coords(w, h, l.second);
        cxy n2(w, h);

        std::cout << "l " << l.first->ID()
                  << " " << l.second->ID()
                  << " " << sqrt(n1.dist2(n2))
                  << "\n";
    }
    for (int kn1 = 0; kn1 < vN.size(); kn1++)
        for (int kn2 = kn1 + 1; kn2 < vN.size(); kn2++)
        {
            bool found = false;
            for (auto &l : vL)
                if ((l.first == vN[kn1] && l.second == vN[kn2]) || (l.first == vN[kn2] && l.second == vN[kn1]))
                {
                    found = true;
                    break;
                }
            if (found)
                continue;

            int w1, h1, w2, h2;
            A->coords(w1, h1, vN[kn1]);
            A->coords(w2, h2, vN[kn2]);
            if (isBlocked(w1, h1, w2, h2))
                continue;

            // missing link

            std::cout << "l " << vN[kn1]->ID()
                      << " " << vN[kn2]->ID()
                      << " " << nx * ny
                      << "\n";
        }
}

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
std::vector<cOCell *> cObstacle::adjacent(cOCell *n)
{
    std::vector<cOCell *> ret;
    for (auto &l : vL)
    {
        if (l.first == n)
            ret.push_back(l.second);
        if (l.second == n)
            ret.push_back(l.first);
    }
    return ret;
}
void cObstacle::spanningTree()
{

    // add initial arbitrary link
    auto v = vN[0];
    auto w = *adjacent(v).begin();
    mySpanningTree.push_back(std::make_pair(v, w));
    v->fvisited = true;
    w->fvisited = true;

    // while nodes remain outside of span
    while (1)
    {
        // int v;      // node in span
        // int w = -1; // node not yet in span
        int min_cost = INT_MAX;
        std::pair<cOCell *, cOCell *> bestLink;

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
                    if ((l.first == v && l.second == w) || (l.first == w && l.second == v))
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
        mySpanningTree.push_back( bestLink );
        bestLink.first->fvisited = true;
        bestLink.second->fvisited = true;
    }
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
                for (auto &l : myObstacle.links())
                {
                    int w, h, w2, h2;
                    grid->coords(
                        w, h, l.first);
                    // std::cout << w << " " << h << " -> ";
                    grid->coords(
                        w2, h2, l.second);
                    // std::cout << w2 << " " << h2 << "\n";
                    S.line({20 * w, 20 * h, 20 * w2, 20 * h2});
                }
                S.color(0x0000FF);
                S.penThick(2);
                for (auto &pl : myObstacle.spanningTree_get())
                {
                    int w, h, w2, h2;
                    grid->coords(
                        w, h, pl.first);
                    grid->coords(
                        w2, h2, pl.second);
                    S.line({20 * w, 20 * h, 20 * w2, 20 * h2});
                }
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
