#include <boost/test/unit_test.hpp>
#include <sonar_localisation/Dummy.hpp>

using namespace sonar_localisation;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    sonar_localisation::DummyClass dummy;
    dummy.welcome();
}
