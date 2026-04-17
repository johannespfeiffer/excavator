#include "app.h"

#include <assert.h>

int main(void)
{
    const int result = app_run();

    assert(result == 0);
    return 0;
}
