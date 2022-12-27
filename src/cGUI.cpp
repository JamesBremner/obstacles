#include <string>
#include "wex.h"
#include <autocell.h>
#include "cxy.h"
#include "cObstacle.h"
#include "cGUI.h"

void cGUI::draw(PAINTSTRUCT &ps)
{
    if (myObstacle.view() == -999)
        return;

    const int scale = 10;

    wex::shapes S(ps);
    int W, H;
    myObstacle.size(W, H);

    // draw the obstacles
    S.color(0x0000FF);
    for (int h = 0; h < H; h++)
        for (int w = 0; w < W; w++)
        {
            auto s = myObstacle.draw(w, h);
            if (s.empty())
                continue;
            if (s == "X")
            {
                S.color(0x0000FF);
                S.text(s, {w * scale, h * scale});
                continue;
            }
            S.color(0x000000);
            S.text(s, {w * scale, h * scale});
        }

    auto grid = myObstacle.grid();
    S.color(0x000000);
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

    std::stringstream sspath;
    int pathCount = 0;
    switch (myViewType)
    {
    case eView::route:
        S.color(0xFF0000);
        S.penThick(1);
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
            S.line({scale * w, scale * h, scale * w2, scale * h2});


            sspath << " -> " << n2->ID();
            pathCount++;
            if( pathCount > 15 ) {
                sspath << "\r\n";
                pathCount = 0;
            }
        }
        S.text(sspath.str(), {scale, H * scale, 1000, 1000});
        // S.text( 
        //     "Nodes revisited " +
        //     std::to_string(myObstacle.NodesRevisited().size()),
        //     {10,H*scale+30});
        std::cout << "Nodes revisited " << myObstacle.NodesRevisited().size() << " : ";
        for( auto n : myObstacle.NodesRevisited() )
            std::cout << n->ID() << " ";
        break;

    case eView::span:
        S.color(0x0000FF);
        S.penThick(2);
        for (auto &pl : myObstacle.spanningTree_get())
        {
            int w, h, w2, h2;
            grid->coords(
                w, h, std::get<0>(pl));
            grid->coords(
                w2, h2, std::get<1>(pl));
            S.line({scale * w, scale * h, scale * w2, scale * h2});
        }
        break;
    }
}
