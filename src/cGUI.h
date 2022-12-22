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
              {50, 50, 1000, 700}),
          lb(wex::maker::make<wex::label>(fm)),
          myViewType(eView::route)
    {
        ConstructMenu();

        fm.events().draw(
            [this](PAINTSTRUCT &ps)
            {
                draw( ps );
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

    void draw(PAINTSTRUCT &ps);
};
