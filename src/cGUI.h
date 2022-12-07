class cStarterGUI
{
public:
    /** CTOR
     * @param[in] title will appear in application window title
     * @param[in] vlocation set location and size of appplication window
     *
     * Usage:
     *
     * <pre>
class appGUI : public cStarterGUI
{
public:
    appGUI()
        : cStarterGUI(
              "The Appliccation",
              {50, 50, 1000, 500})
    {

        // initialize the solution
        ...

        show();
        run();
    }
    </pre>
    */
    cStarterGUI(
        const std::string &title,
        const std::vector<int> &vlocation)
        : fm(wex::maker::make())
    {
        fm.move(vlocation);
        fm.text(title);

        fm.events().draw(
            [&](PAINTSTRUCT &ps)
            {
                wex::shapes S(ps);
                draw(S);
            });
    }
    /** Draw nothing
     *
     * An application should over-ride this method
     * to perform any drawing reuired
     */
    virtual void draw(wex::shapes &S)
    {
    }
    void show()
    {
        fm.show();
    }
    void run()
    {
        fm.run();
    }

protected:
    wex::gui &fm;
};

class cGUI : public cStarterGUI
{
public:
    cGUI()
        : cStarterGUI(
              "Obstacles",
              {50, 50, 1000, 500}),
          lb(wex::maker::make<wex::label>(fm)),
          myViewType(eView::route)
    {
        ConstructMenu();

        fm.events().draw(
            [&](PAINTSTRUCT &ps)
            {
                if (myObstacle.view() == -999)
                    return;
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
                            S.text(s, {w * 20, h * 20});
                            continue;
                        }
                        S.color(0x000000);
                        S.text(s, {w * 20, h * 20});
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
                        S.line({20 * w, 20 * h, 20 * w2, 20 * h2});
                        sspath << " -> " << n2->ID();
                    }
                    S.text(sspath.str(), {20, H * 20});
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
                        S.line({20 * w, 20 * h, 20 * w2, 20 * h2});
                    }
                    break;
                }
            });

        show();
        run();
    }

private:
    wex::label &lb;
    cObstacle myObstacle;
    enum class eView
    {
        route,
        span,
    };
    eView myViewType;

    void ConstructMenu();
};
