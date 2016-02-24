#include <boost/test/unit_test.hpp>
#include <imaging_sonar_localization/Dummy.hpp>

using namespace imaging_sonar_localization;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    imaging_sonar_localization::DummyClass dummy;
    dummy.welcome();
}
