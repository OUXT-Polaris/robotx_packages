#include <passthrough_filter.h>

passthough_filter::passthough_filter() : param_(passthough_filter::parameters())
{

}

passthough_filter::passthough_filter(passthough_filter::parameters& param) : param_(param)
{

}