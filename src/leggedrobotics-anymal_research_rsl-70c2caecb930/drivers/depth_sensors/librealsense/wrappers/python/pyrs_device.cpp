/* License: Apache 2.0. See LICENSE file in root directory.
Copyright(c) 2017 Intel Corporation. All Rights Reserved. */

#include "python.hpp"
#include "../include/any_librealsense2/hpp/rs_device.hpp"
#include "../include/any_librealsense2/hpp/rs_record_playback.hpp" // for downcasts

void init_device(py::module &m) {
    /** rs_device.hpp **/
    py::class_<rs2::device> device(m, "device"); // No docstring in C++
    device.def("query_sensors", &rs2::device::query_sensors, "Returns the list of adjacent devices, "
               "sharing the same physical parent composite device.")
        .def_property_readonly("sensors", &rs2::device::query_sensors, "List of adjacent devices, "
                               "sharing the same physical parent composite device. Identical to calling query_sensors.")
        .def("first_depth_sensor", [](rs2::device& self) { return self.first<rs2::depth_sensor>(); }) // No docstring in C++
        .def("first_roi_sensor", [](rs2::device& self) { return self.first<rs2::roi_sensor>(); }) // No docstring in C++
        .def("first_pose_sensor", [](rs2::device& self) { return self.first<rs2::pose_sensor>(); }) // No docstring in C++
        .def("supports", &rs2::device::supports, "Check if specific camera info is supported.", "info"_a)
        .def("get_info", &rs2::device::get_info, "Retrieve camera specific information, "
             "like versions of various internal components", "info"_a)
        .def("hardware_reset", &rs2::device::hardware_reset, "Send hardware reset request to the device")
        .def(py::init<>())
        .def("__nonzero__", &rs2::device::operator bool)
        .def(BIND_DOWNCAST(device, debug_protocol))
        .def(BIND_DOWNCAST(device, playback))
        .def(BIND_DOWNCAST(device, recorder))
        .def(BIND_DOWNCAST(device, tm2))
        .def(BIND_DOWNCAST(device, updatable))
        .def(BIND_DOWNCAST(device, update_device))
        .def("__repr__", [](const rs2::device &self) {
            std::stringstream ss;
            ss << "<" SNAME ".device: " << self.get_info(RS2_CAMERA_INFO_NAME)
                << " (S/N: " << self.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                << ")>";
            return ss.str();
        });

    // not binding update_progress_callback, templated

    py::class_<rs2::updatable, rs2::device> updatable(m, "updatable"); // No docstring in C++
    updatable.def(py::init<rs2::device>())
        .def("enter_update_state", &rs2::updatable::enter_update_state, "Move the device to update state, this will cause the updatable device to disconnect and reconnect as an update device.")
        .def("create_flash_backup", (std::vector<uint8_t>(rs2::updatable::*)() const) &rs2::updatable::create_flash_backup,
             "Create backup of camera flash memory. Such backup does not constitute valid firmware image, and cannot be "
             "loaded back to the device, but it does contain all calibration and device information.")
        .def("create_flash_backup", [](rs2::updatable& self, std::function<void(float)> f) { return self.create_flash_backup(f); },
             "Create backup of camera flash memory. Such backup does not constitute valid firmware image, and cannot be "
             "loaded back to the device, but it does contain all calibration and device information.",
             "callback"_a)
        .def("update_unsigned", (void(rs2::updatable::*)(const std::vector<uint8_t>&, int) const) &rs2::updatable::update_unsigned,
             "Update an updatable device to the provided unsigned firmware. This call is executed on the caller's thread.", "fw_image"_a, "update_mode"_a = RS2_UNSIGNED_UPDATE_MODE_UPDATE)
        .def("update_unsigned", [](rs2::updatable& self, const std::vector<uint8_t>& fw_image, std::function<void(float)> f, int update_mode) { return self.update_unsigned(fw_image, f, update_mode); },
             "Update an updatable device to the provided unsigned firmware. This call is executed on the caller's thread and it supports progress notifications via the callback.",
             "fw_image"_a, "callback"_a, "update_mode"_a = RS2_UNSIGNED_UPDATE_MODE_UPDATE);

    py::class_<rs2::update_device, rs2::device> update_device(m, "update_device");
    update_device.def(py::init<rs2::device>())
        .def("update", [](rs2::update_device& self, const std::vector<uint8_t>& fw_image) { return self.update(fw_image); },
             "Update an updatable device to the provided firmware. This call is executed on the caller's thread.", "fw_image"_a)
        .def("update", [](rs2::update_device& self, const std::vector<uint8_t>& fw_image, std::function<void(float)> f) { return self.update(fw_image, f); },
             "Update an updatable device to the provided firmware. This call is executed on the caller's thread and it supports progress notifications via the callback.",
             "fw_image"_a, "callback"_a);

    py::class_<rs2::debug_protocol> debug_protocol(m, "debug_protocol"); // No docstring in C++
    debug_protocol.def(py::init<rs2::device>())
        .def("send_and_receive_raw_data", &rs2::debug_protocol::send_and_receive_raw_data,
             "input"_a);  // No docstring in C++

    py::class_<rs2::device_list> device_list(m, "device_list"); // No docstring in C++
    device_list.def(py::init<>())
        .def("contains", &rs2::device_list::contains) // No docstring in C++
        .def("__getitem__", [](const rs2::device_list& self, size_t i) {
            if (i >= self.size())
                throw py::index_error();
            return self[uint32_t(i)];
        })
        .def("__len__", &rs2::device_list::size)
        .def("size", &rs2::device_list::size) // No docstring in C++
        .def("__iter__", [](const rs2::device_list& self) {
            return py::make_iterator(self.begin(), self.end());
        }, py::keep_alive<0, 1>())
        .def("__getitem__", [](const rs2::device_list& self, py::slice slice) {
            size_t start, stop, step, slicelength;
            if (!slice.compute(self.size(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();
            auto *dlist = new std::vector<rs2::device>(slicelength);
            for (size_t i = 0; i < slicelength; ++i) {
                (*dlist)[i] = self[uint32_t(start)];
                start += step;
            }
            return dlist;
        })
        .def("front", &rs2::device_list::front) // No docstring in C++
        .def("back", &rs2::device_list::back); // No docstring in C++

    py::class_<rs2::tm2, rs2::device> tm2(m, "tm2"); // No docstring in C++
    tm2.def(py::init<rs2::device>(), "device"_a)
        .def("enable_loopback", &rs2::tm2::enable_loopback, "Enter the given device into "
             "loopback operation mode that uses the given file as input for raw data", "filename"_a)
        .def("disable_loopback", &rs2::tm2::disable_loopback, "Restores the given device into normal operation mode")
        .def("is_loopback_enabled", &rs2::tm2::is_loopback_enabled, "Checks if the device is in loopback mode or not")
        .def("connect_controller", &rs2::tm2::connect_controller, "Connects to a given tm2 controller", "mac_address"_a)
        .def("disconnect_controller", &rs2::tm2::disconnect_controller, "Disconnects a given tm2 controller", "id"_a)
        .def("set_intrinsics", &rs2::tm2::set_intrinsics, "Set camera intrinsics", "sensor_id"_a, "intrinsics"_a)
        .def("set_extrinsics", &rs2::tm2::set_extrinsics, "Set camera extrinsics", "from_stream"_a, "from_id"_a, "to_stream"_a, "to_id"_a, "extrinsics"_a)
        .def("set_motion_device_intrinsics", &rs2::tm2::set_motion_device_intrinsics, "Set motion device intrinsics", "stream_type"_a, "motion_intrinsics"_a)
        .def("reset_to_factory_calibration", &rs2::tm2::reset_to_factory_calibration, "Reset to factory calibration")
        .def("write_calibration", &rs2::tm2::write_calibration, "Write calibration to device's EEPROM");

    /** end rs_device.hpp **/
}
