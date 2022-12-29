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
}

main()
{
    return raven::set::UnitTest::RunAllTests();
}