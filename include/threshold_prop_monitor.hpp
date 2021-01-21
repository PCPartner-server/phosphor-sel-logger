#pragma once
#include <sdbusplus/asio/object_server.hpp>
#include <sel_logger.hpp>
#include <sensorutils.hpp>
#include <threshold_event_monitor.hpp>
#include <phosphor-logging/elog.hpp>
#include <sdbusplus/exception.hpp>

//code from phosphor-led-manager
constexpr auto LED_SERVICE= "xyz.openbmc_project.LED.GroupManager";
//TODO: if led group interface string is constant, why would led service string be otherwise?
constexpr auto LED_GROUPS = "/xyz/openbmc_project/led/groups/";
constexpr auto LED_FAULT = "fault";
//TODO: map this to the configure.ac

void triggerFaultLeds(sdbusplus::asio::connection& conn, const std::string& path, bool assert)
{
    auto pos = path.rfind("/");
    if (pos == std::string::npos)
    {
        return;
    }
    auto unit = path.substr(pos + 1);

    std::string ledPath = LED_GROUPS + unit + '_' + LED_FAULT;
#if DEBUG
std::printf("%s %d ledPath=%s\n",__func__,__LINE__,ledPath.c_str());
std::printf("%s %d assert=%s\n",__func__,__LINE__,assert ? "true" : "false");
#endif
    auto method = conn.new_method_call(LED_SERVICE, ledPath.c_str(),
                                      "org.freedesktop.DBus.Properties", "Set");
    method.append("xyz.openbmc_project.Led.Group");
    method.append("Asserted");

    method.append(std::variant<bool>(assert));

    try
    {
        conn.call_noreply(method);
    }
    catch (const sdbusplus::exception::SdBusError& e)
    {
        std::printf("Asserting led %s has failed with reason: %s\n",ledPath.c_str(),e.what());
    }

    return;
}

inline static sdbusplus::bus::match::match
    startThresholdEventMonitor(std::shared_ptr<sdbusplus::asio::connection> conn)
{
    auto thresholdEventMatcherCallback = [conn](sdbusplus::message::message& msg) {
        std::string sensorName;
        std::string thresholdInterface;
        boost::container::flat_map<std::string, std::variant<bool>>
            propertiesChanged;
        static boost::container::flat_set<std::pair<std::string, std::string>>
            assertedEvents;
        std::vector<uint8_t> eventData(selEvtDataMaxSize,
                                       selEvtDataUnspecified);

        msg.read(sensorName, propertiesChanged);
        if (propertiesChanged.empty())
        {
            return;
        }

        std::string event = propertiesChanged.begin()->first;
        auto assert =
            std::get<bool>(propertiesChanged.begin()->second);
        double assertValue;

        if (event.empty())
        {
            return;
        }
        //sensorName = msg.get_path();
        thresholdInterface = sensorName;
#if DEBUG
std::printf("%s %d path=%s event=%s thresholdInterface=%s sensorName=%s\n",__func__,__LINE__,msg.get_path(),event.c_str(),thresholdInterface.c_str(),sensorName.c_str());
#endif
        // Check the asserted events to determine if we should log this event
        std::pair<std::string, std::string> pathAndEvent(
            std::string(msg.get_path()), event);
        if (assert)
        {
            // For asserts, add the event to the set and only log it if it's new
            if (assertedEvents.insert(pathAndEvent).second == false)
            {
                // event is already in the set
                return;
            }
        }
        else
        {
            // For deasserts, remove the event and only log the deassert if it
            // was asserted
            if (assertedEvents.erase(pathAndEvent) == 0)
            {
                // asserted event was not in the set
                return;
            }
        }

        // Set the IPMI threshold event type based on the event details from the
        // message
        if (event == "CriticalAlarmLow")
        {
            eventData[0] =
                static_cast<uint8_t>(thresholdEventOffsets::lowerCritGoingLow);
        }
        else if (event == "WarningAlarmLow")
        {
            eventData[0] = static_cast<uint8_t>(
                thresholdEventOffsets::lowerNonCritGoingLow);
        }
        else if (event == "WarningAlarmHigh")
        {
            eventData[0] = static_cast<uint8_t>(
                thresholdEventOffsets::upperNonCritGoingHigh);
        }
        else if (event == "CriticalAlarmHigh")
        {
            //we only care about critical high
            triggerFaultLeds(*conn,msg.get_path(),assert);
            eventData[0] =
                static_cast<uint8_t>(thresholdEventOffsets::upperCritGoingHigh);
        }
        // Indicate that bytes 2 and 3 are threshold sensor trigger values
        eventData[0] |= thresholdEventDataTriggerReadingByte2 |
                        thresholdEventDataTriggerReadingByte3;

        // Get the sensor reading to put in the event data
        sdbusplus::message::message getSensorValue =
            conn->new_method_call(msg.get_sender(), msg.get_path(),
                                  "org.freedesktop.DBus.Properties", "GetAll");
        getSensorValue.append("xyz.openbmc_project.Sensor.Value");
        boost::container::flat_map<std::string, std::variant<double, int64_t>>
            sensorValue;
        try
        {
            sdbusplus::message::message getSensorValueResp =
                conn->call(getSensorValue);
            getSensorValueResp.read(sensorValue);
        }
        catch (sdbusplus::exception_t&)
        {
            std::cerr << "error getting sensor value from " << msg.get_path()
                      << "\n";
            return;
        }
        double max = 0;
        auto findMax = sensorValue.find("MaxValue");
        if (findMax != sensorValue.end())
        {
            max = std::visit(ipmi::VariantToDoubleVisitor(), findMax->second);
        }
        double min = 0;
        auto findMin = sensorValue.find("MinValue");
        if (findMin != sensorValue.end())
        {
            min = std::visit(ipmi::VariantToDoubleVisitor(), findMin->second);
        }
        auto findValue = sensorValue.find("Value");
        if (findValue != sensorValue.end())
        {
            assertValue = std::visit(ipmi::VariantToDoubleVisitor(), findValue->second);
        }
#if DEBUG
std::printf("%s %d min=%f max=%f assertValue=%f\n",__func__,__LINE__,min,max,assertValue);
#endif
        try
        {
            eventData[1] = ipmi::getScaledIPMIValue(assertValue, max, min);
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what();
            eventData[1] = selEvtDataUnspecified;
        }

        // Get the threshold value to put in the event data
        // Get the threshold parameter by removing the "Alarm" text from the
        // event string
        std::string alarm("Alarm");
        if (std::string::size_type pos = event.find(alarm);
            pos != std::string::npos)
        {
            event.erase(pos, alarm.length());
        }
#if DEBUG
std::printf("%s %d event=%s min=%f max=%f thresholdInterface=%s \n",__func__,__LINE__,event.c_str(),min,max,thresholdInterface.c_str());
#endif
        sdbusplus::message::message getThreshold =
            conn->new_method_call(msg.get_sender(), msg.get_path(),
                                  "org.freedesktop.DBus.Properties", "Get");
        getThreshold.append(thresholdInterface, event);
        std::variant<double, int64_t> thresholdValue;

        try
        {
            sdbusplus::message::message getThresholdResp =
                conn->call(getThreshold);
            getThresholdResp.read(thresholdValue);
        }
        catch (sdbusplus::exception_t&)
        {
            std::cerr << "error getting sensor threshold from "
                      << msg.get_path() << "\n";
            return;
        }
        double thresholdVal =
            std::visit(ipmi::VariantToDoubleVisitor(), thresholdValue);
#if DEBUG
std::printf("%s %d min=%f max=%f thresholdInterface=%s thresholdVal=%f\n",__func__,__LINE__,min,max,thresholdInterface.c_str(),thresholdVal);
#endif
        double scale = 0;
        auto findScale = sensorValue.find("Scale");
        if (findScale != sensorValue.end())
        {
            scale =
                std::visit(ipmi::VariantToDoubleVisitor(), findScale->second);
            thresholdVal *= std::pow(10, scale);
        }
        try
        {
            eventData[2] = ipmi::getScaledIPMIValue(thresholdVal, max, min);
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what();
            eventData[2] = selEvtDataUnspecified;
        }
#if DEBUG
std::printf("%s %d min=%f max=%f thresholdInterface=%s thresholdVal=%f scale=%f event=%s\n",__func__,__LINE__,min,max,thresholdInterface.c_str(),thresholdVal,scale,event.c_str());
#endif
        std::string threshold;
        std::string direction;
        std::string redfishMessageID =
            "OpenBMC." + openBMCMessageRegistryVersion;
        if (event == "CriticalLow")
        {
            threshold = "critical low";
            if (assert)
            {
                direction = "low";
                redfishMessageID += ".SensorThresholdCriticalLowGoingLow";
            }
            else
            {
                direction = "high";
                redfishMessageID += ".SensorThresholdCriticalLowGoingHigh";
            }
        }
        else if (event == "WarningLow")
        {
            threshold = "warning low";
            if (assert)
            {
                direction = "low";
                redfishMessageID += ".SensorThresholdWarningLowGoingLow";
            }
            else
            {
                direction = "high";
                redfishMessageID += ".SensorThresholdWarningLowGoingHigh";
            }
        }
        else if (event == "WarningHigh")
        {
            threshold = "warning high";
            if (assert)
            {
                direction = "high";
                redfishMessageID += ".SensorThresholdWarningHighGoingHigh";
            }
            else
            {
                direction = "low";
                redfishMessageID += ".SensorThresholdWarningHighGoingLow";
            }
        }
        else if (event == "CriticalHigh")
        {
            threshold = "critical high";
            if (assert)
            {
                direction = "high";
                redfishMessageID += ".SensorThresholdCriticalHighGoingHigh";
            }
            else
            {
                direction = "low";
                redfishMessageID += ".SensorThresholdCriticalHighGoingLow";
            }
        }
#if DEBUG
std::printf("%s %d path=%s\n",__func__,__LINE__,msg.get_path());
std::printf("%s %d sensorName=%s\n",__func__,__LINE__,sensorName.c_str());
std::printf("%s %d direction=%s redfishMessageID=%s\n",__func__,__LINE__,direction.c_str());
std::printf("%s %d redfishMessageID=%s\n",__func__,__LINE__,redfishMessageID.c_str());
std::printf("%s %d assertValue=%f thresholdVal=%f\n",__func__,__LINE__,assertValue,thresholdVal);
#endif
        std::string journalMsg(std::string(msg.get_path()) + " sensor crossed a " +
                               threshold + " threshold going " + direction +
                               ". Reading=" + std::to_string(assertValue) +
                               " Threshold=" + std::to_string(thresholdVal) +
                               ".");
#if DEBUG
std::printf("%s %d journalMsg=%s\n",__func__,__LINE__,journalMsg.c_str());
#endif
        selAddSystemRecord(
            journalMsg, std::string(msg.get_path()), eventData, assert,
            selBMCGenID, "REDFISH_MESSAGE_ID=%s", redfishMessageID.c_str(),
            "REDFISH_MESSAGE_ARGS=%.*s,%f,%f", strlen(msg.get_path()),
            msg.get_path(), assertValue, thresholdVal);
    };

    sdbusplus::bus::match::match thresholdEventMatcher(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0namespace='xyz.openbmc_project.Sensor.Threshold'",
        std::move(thresholdEventMatcherCallback));

    return thresholdEventMatcher;
}
