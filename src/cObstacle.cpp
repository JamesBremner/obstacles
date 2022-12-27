#include <vector>
#include <algorithm>
#include <iostream>
#include "cxy.h"
#include <autocell.h>
#include "cObstacle.h"

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

    int bestCountRevisited = 1e7;
    vlink_t bestPath;
    std::vector<cOCell *> bestNodesRevisited;
    int bestStart;
    for (int start = 0; start < connectedLeaves.size(); start++)
    // int start = 30;
    {
        tour(connectedLeaves, start);

        if (myNodesRevisited.size() < bestCountRevisited)
        {
            bestCountRevisited = myNodesRevisited.size();
            bestPath = vPath;
            bestNodesRevisited = myNodesRevisited;
            bestStart = start;
            if (!bestCountRevisited)
                break;
        }
    }
    vPath = bestPath;
    myNodesRevisited = bestNodesRevisited;
}

void cObstacle::tour(
    vlink_t &connectedLeaves,
    int start)
{
    vPath.clear();
    myNodesRevisited.clear();
    for (auto n : vN)
        n->fvisited = false;

    link_t best_link;

    auto v = std::get<0>(vL[start]);
    // std::cout << v->ID() << " ";
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
            if (v->ID() == 382)
            {
                int ibg = 0;
            }
            // std::cout << " jump path ( " << jump_path[0]->ID() <<" ";
            for (auto k = 1; k < jump_path.size(); k++)
            {
                // std::cout << jump_path[k]->ID()   <<" ";
                pathAdd(
                    jump_path[k - 1],
                    jump_path[k]);
            }
            // std::cout << " )\n";
            v = w;
        }
    }
}

void cObstacle::pathAdd(
    cOCell *node1,
    cOCell *node2)
{
    vPath.push_back(std::make_tuple(node1, node2, 0));
    if (node2->fvisited)
    {
        myNodesRevisited.push_back(node2);
    }
    node2->fvisited = true;
    // std::cout << node2->ID() << " ";
}
cOCell *cObstacle::ClosestUnvisited(
    cOCell *startp,
    vlink_t &vlink,
    std::vector<cOCell *> &path)
{
    // std::cout << "ClosestUnvisited from " << startp->ID() << "\n";

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
    if (bestDist == INT_MAX)
        return best;

    path.push_back(best);
    int i = best_index;
    while (1)
    {
        i = pred[i];
        path.push_back(vN[i]);
        if (i == startIdx)
            break;
    }
    std::reverse(path.begin(), path.end());

    return best;
}
