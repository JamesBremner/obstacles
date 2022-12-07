

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

    bool fvisited;      ///< true if cell has been visited by the robot's path
};

typedef std::tuple<cOCell *, cOCell *, double> link_t;
typedef std::vector<link_t> vlink_t;

class cObstacle
{

    int nx, ny;                  ///< grid size

    /**
     * @brief radius of robot's coverage
     * 
     * >0 maximum distance, diagonal or orthogonal, in grid points
     * -1 the obstacles are spaced 2 grid points apart, with room for the robot to pass between
     * -999 initial, unset value
     * 
     */
    int myView;                   ///< view radius

    cell::cAutomaton<cOCell> *A; ///< 2D grid
    std::vector<cOCell *> vN;    ///< nodes to be included in path
    vlink_t vL;                  ///< links between nodes
    vlink_t vPath;
    vlink_t mySpanningTree;
    bool myfrect;               /// true if grid is rectangular
    std::vector< cxy > myPolygon;   /// polygon vertices for non-rectangular grid

public:

    cObstacle()
    : myView( -999 )
    , myfrect( true )
    {}

    void clear();

    /// @brief set grid size
    /// @param x 
    /// @param y 
    void grid( int x, int y );

    /// @brief set grid to be not rectangular
    void poly()
    {
        myfrect = false;
    }

    /// @brief Add vertex to polgon arounf grid
    /// @param p 
    void polyAdd( const cxy& p )
    {
        myPolygon.push_back( p );
    }

    /// @brief set robot view radius
    /// @param v 
    void view( int v )
    {
        myView = v;
    }

    /// @brief get robot view radius
    /// @return 
    int view() const
    {
        return myView;
    }

    /// @brief Set obstacle at location
    /// @param x 
    /// @param y 
    void obstacle( int x, int y );

    /// @brief construct nodes to be visited
    void unobstructedPoints();

    /// @brief connect nodes, avoiding obstacles
    void connect();

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

    /// @brief Find closest unvisited node
    /// @param start start node
    /// @param vlink allowed links
    /// @param path path from start to nearest
    /// @return pointer to nearest unvisited node
    ///
    /// Uses Dijsktra alorithm
    cOCell *ClosestUnvisited(
        cOCell *start,
        vlink_t &vlink,
        std::vector<cOCell *> &path);

    /// @brief Add connection to path
    /// @param node1 
    /// @param node2 
    void pathAdd( 
        cOCell * node1,
        cOCell * node2   );
};

    /// @brief read layout of obstacles from file
    /// @param fname
    void read(
        cObstacle& obs,
        const std::string &fname);
