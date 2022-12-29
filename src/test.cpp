#include <iostream>
#include "cutest.h"
#include "cxy.h"
#include <autocell.h>
#include "cObstacle.h"
TEST(test)
{
    cObstacle O;
    read(O, "../data/data.txt");
    O.unobstructedPoints();
    O.connect();
    O.tourSpanningTree();
    CHECK_EQUAL( 15, O.path().size() );
    CHECK_EQUAL( 0, O.NodesRevisited().size() );
}
TEST(farm)
{
    cObstacle O;
    read(O, "../data/farm.txt");
    O.unobstructedPoints();
    O.connect();
    O.tourSpanningTree();
    CHECK_EQUAL( 99, O.path().size() );
     CHECK_EQUAL( 0, O.NodesRevisited().size() );
}
TEST(farmpoly)
{
    cObstacle O;
    read(O, "../data/farmpoly.txt");
    O.unobstructedPoints();
    O.connect();
    O.tourSpanningTree();
    CHECK_EQUAL( 88, O.path().size() );
     CHECK_EQUAL( 0, O.NodesRevisited().size() );
}
TEST(farmconcave)
{
    cObstacle O;
    read(O, "../data/farmconcave.txt");
    O.unobstructedPoints();
    O.connect();
    O.tourSpanningTree();
    CHECK_EQUAL( 82, O.path().size() );
    CHECK_EQUAL( 1, O.NodesRevisited().size() );
}

main()
{
    return raven::set::UnitTest::RunAllTests();
}