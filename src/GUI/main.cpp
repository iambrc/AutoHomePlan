#include <stdexcept>

#include "GUI/window.h"

int main()
{
    try
    {
        Window w("AutoHomePlan");
        if (!w.init())
            return 1;

        w.run();
        return 0;
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}