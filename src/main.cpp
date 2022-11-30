#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <wex.h>
#include "cStarterGUI.h"
#include <autocell.h>

class cxy
{

public:
    double x;
    double y;

    cxy()
    {}
    cxy( int X, int Y )
    : x( X ), y( Y )
    {}
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
    cxy v = vect( other );
    return v.x*v.x + v.y*v.y;
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
    return dist2( closest );
}

class cOCell : public cell::cCell
{
public:
    cOCell()
        : cell::cCell(), myType(0)
    {
    }

    int myType;
};

class cObstacle
{
    int nx, ny, view;
    cell::cAutomaton<cOCell> *A;
    std::vector<cOCell *> vN;
    std::vector<std::pair<cOCell *, cOCell *>> vL;

public:
    void read(const std::string &fname);
    void unobstructedPoints();
    void connect();
    void output();
    std::string draw(int w, int h) const;
    void size(int &w, int &h) const
    {
        w = nx;
        h = ny;
    }
    std::vector<std::pair<cOCell *, cOCell *>> links()
    {
        return vL;
    }
    cell::cAutomaton<cOCell> *grid()
    {
        return A;
    }
    bool isBlocked(int x1, int y1, int x2, int y2);
};

void cObstacle::read(const std::string &fname)
{
    std::ifstream ifs(fname);
    if (!ifs.is_open())
        exit(1);
    ifs >> nx >> ny >> view;
    A = new cell::cAutomaton<cOCell>(nx, ny);
    while (ifs.good())
    {
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
            if (A->cell(w, h)->myType != 1)
                continue;
            int ow, oh;
            A->coords(ow, oh, A->cell(w, h));

            cxy obstacle( ow, oh );
            cxy line1( x1, y1 );
            cxy line2( x2,y2);
            if( obstacle.dis2toline(line1,line2) < 2 )
                return true;
        }
    return false;
}

void cObstacle::connect()
{
    for (auto n1 : vN)
        for (auto n2 : vN)
        {
            // check for self connections
            if (n1 == n2)
                continue;

            // check for non-neighbours
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

void cObstacle::output()
{
    for( auto n : vN ) {
        std::cout << "n " << n->ID() << "\n";
    }
    for( auto l : vL )
    {
        if(  l.first->ID() >  l.second->ID())
            continue;
        int w,h;
        A->coords( w,h,l.first);
        cxy n1( w, h );
        A->coords( w,h,l.second);
        cxy n2( w, h );

        std::cout << "l " << l.first->ID() 
            <<" "<< l.second->ID() 
            <<" "<< sqrt( n1.dist2( n2 ))
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
        return " ";
    case 1:
        return "X";
    case 2:
        return " ";
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
        myObstacle.output();

        fm.events().draw(
            [&](PAINTSTRUCT &ps)
            {
                wex::shapes S(ps);
                int W, H;
                myObstacle.size(W, H);
                for (int h = 0; h < H; h++)
                    for (int w = 0; w < W; w++)

                        S.text(
                            myObstacle.draw(w, h),
                            {w * 20, h * 20});
                auto grid = myObstacle.grid();
                for (auto &l : myObstacle.links())
                {
                    int w, h, w2, h2;
                    grid->coords(
                        w, h, l.first);
                    //std::cout << w << " " << h << " -> ";
                    grid->coords(
                        w2, h2, l.second);
                    //std::cout << w2 << " " << h2 << "\n";
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
