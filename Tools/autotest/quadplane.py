#!/usr/bin/env python

# Fly ArduPlane QuadPlane in SITL
from __future__ import print_function
import os
from pymavlink import mavutil
import math

from common import AutoTest
from common import AutoTestTimeoutException, NotAchievedException

from pysim import vehicleinfo
import operator


# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-27.274439, 151.290064, 343, 8.7)
MISSION = 'ArduPlane-Missions/Dalby-OBC2016.txt'
FENCE = 'ArduPlane-Missions/Dalby-OBC2016-fence.txt'
WIND = "0,180,0.2"  # speed,direction,variance


class AutoTestQuadPlane(AutoTest):
    @staticmethod
    def get_not_armable_mode_list():
        return []

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return []

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return []

    @staticmethod
    def get_normal_armable_modes_list():
        return []

    def default_frame(self):
        return "quadplane"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def log_name(self):
        return "QuadPlane"

    def apply_defaultfile_parameters(self):
        # plane passes in a defaults_file in place of applying
        # parameters afterwards.
        pass

    def defaults_filepath(self):
        vinfo = vehicleinfo.VehicleInfo()
        defaults_file = vinfo.options["ArduPlane"]["frames"][self.frame]["default_params_filename"]
        if isinstance(defaults_file, str):
            defaults_file = [defaults_file]
        defaults_list = []
        for d in defaults_file:
            defaults_list.append(os.path.join(testdir, d))
        return ','.join(defaults_list)

    def is_plane(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("LAND_DISARMDELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("LAND_DISARMDELAY", delay)

    def test_motor_mask(self):
        """Check operation of output_motor_mask"""
        """copter tailsitters will add condition: or (int(self.get_parameter('Q_TAILSIT_MOTMX')) & 1)"""
        if not(int(self.get_parameter('Q_TILT_MASK')) & 1):
            self.progress("output_motor_mask not in use")
            return
        self.progress("Testing output_motor_mask")
        self.wait_ready_to_arm()

        """Default channel for Motor1 is 5"""
        self.progress('Assert that SERVO5 is Motor1')
        assert(33 == self.get_parameter('SERVO5_FUNCTION'))

        modes = ('MANUAL', 'FBWA', 'QHOVER')
        for mode in modes:
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.arm_vehicle()
            self.progress("Raising throttle")
            self.set_rc(3, 1800)
            self.progress("Waiting for Motor1 to start")
            self.wait_servo_channel_value(5, 1100, comparator=operator.gt)

            self.set_rc(3, 1000)
            self.disarm_vehicle()
            self.wait_ready_to_arm()

    def fly_mission(self, filename, fence, height_accuracy=-1):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        self.load_mission(filename)
        self.load_fence(fence)
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.mavproxy.send('mode AUTO\n')
        self.wait_mode('AUTO')
        self.wait_waypoint(1, 19, max_dist=60, timeout=1200)

        self.mav.motors_disarmed_wait()
        # wait for blood sample here
        self.mavproxy.send('wp set 20\n')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(20, 34, max_dist=60, timeout=1200)

        self.mav.motors_disarmed_wait()
        self.progress("Mission OK")

    def fly_qautotune(self):
        self.change_mode("QHOVER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 1800)
        self.wait_altitude(30,
                           40,
                           relative=True,
                           timeout=30)
        self.set_rc(3, 1500)
        self.change_mode("QAUTOTUNE")
        tstart = self.get_sim_time()
        sim_time_expected = 5000
        deadline = tstart + sim_time_expected
        while self.get_sim_time_cached() < deadline:
            now = self.get_sim_time_cached()
            m = self.mav.recv_match(type='STATUSTEXT',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("STATUSTEXT (%u<%u): %s" % (now, deadline, m.text))
            if "AutoTune: Success" in m.text:
                break
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.set_rc(3, 1200)
        self.wait_altitude(-5, 1, relative=True, timeout=30)
        while self.get_sim_time_cached() < deadline:
            self.mavproxy.send('disarm\n')
            try:
                self.wait_text("AutoTune: Saved gains for Roll Pitch Yaw", timeout=0.5)
            except AutoTestTimeoutException as e:
                continue
            break
        self.mav.motors_disarmed_wait()

    def test_pid_tuning(self):
        self.change_mode("FBWA") # we don't update PIDs in MANUAL
        super(AutoTestQuadPlane, self).test_pid_tuning()

    def test_parameter_checks(self):
        self.test_parameter_checks_poscontrol("Q_P")

    def test_precision_landing(self):
        self.pl_set_sitl_params()
        # self.pl_success()
        # self.pl_disabled()
        # self.pl_abort_next_wp()
        # self.pl_proceed_gps()
        # #self.pl_abort_contingency()
        # self.pl_abort_next_wp_lose_target_above_abort_alt()
        # self.pl_abort_next_wp_lose_target_below_abort_alt()
        self.pl_proceed_gps_lose_target_above_abort_alt()
        self.pl_proceed_gps_lose_target_below_abort_alt()


    def pl_set_sitl_params(self):
        self.set_parameter("SIM_PLD_ALT_LMT", 1000)
        self.set_parameter("SIM_PLD_DIST_LMT", 1000)
        self.set_parameter("SIM_PLD_HEIGHT", 300)
        self.set_parameter("SIM_PLD_LAT", -27.27217982)
        self.set_parameter("SIM_PLD_LON", 151.30387505)
        self.set_parameter("SIM_PLD_RATE", 5)
        self.set_parameter("SIM_PLD_TYPE", 0)

    def pl_start_mission(self, filepath):

        # load mission file
        self.load_mission(filepath)

        # takeoff
        self.homeloc = self.mav.location()
        self.mavproxy.send('mode AUTO\n')
        self.wait_mode('AUTO')
        self.mavproxy.send('wp set 0\n')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("SIM_SPEEDUP", 100)
        self.wait_waypoint(1, 3, max_dist=1000, timeout=1200)
        self.set_parameter("SIM_SPEEDUP", 8)

        # end at vtol land wp

    def pl_return(self):

        # set up return leg and wait for landing
        self.set_parameter("SIM_SPEEDUP", 1)
        self.mavproxy.send('wp set 4\n')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("SIM_SPEEDUP", 100)
        self.wait_waypoint(4, 6, max_dist=1000, timeout=1200)
        self.mav.motors_disarmed_wait()
        self.set_parameter("SIM_SPEEDUP", 1)

        
    def pl_success(self):

        print("PRECLAND TEST: Check success in all enabled cases")

        missionlist = ["quadplane_precland_abort_next_wp_mission.txt",
                        "quadplane_precland_proceed_gps_mission.txt",
                        "quadplane-precland_abort_contingency_mission.txt"]

        for mission in missionlist:

            # Simulated target enable
            self.set_parameter("SIM_PLD_ENABLE", 1)
            #self.load_mission("swoop-precland-mission.txt")
            self.pl_start_mission(mission)

            self.mav.motors_disarmed_wait()

            self.assert_visual_landing()

            # return to start
            self.pl_return()

    def pl_disabled(self):

        print("PRECLAND TEST: Precland Disabled")

        # Simulated target enable
        self.set_parameter("SIM_PLD_ENABLE", 1)
        #self.load_mission("swoop-precland-mission.txt")
        self.pl_start_mission("quadplane_precland_disabled_mission.txt")
        
        # wait for landing to be completed
        self.mav.motors_disarmed_wait()

        # check that it didn't land on the target
        self.assert_not_visual_landing()

        # return to start
        self.pl_return()
        

    def pl_abort_next_wp(self):

        print("PRECLAND TEST: Target not found. Abort, continue mission.")

        # Simulated target enable
        self.set_parameter("SIM_PLD_ENABLE", 0)
        #self.load_mission("swoop-precland-mission.txt")
        self.pl_start_mission("quadplane_precland_abort_next_wp_mission.txt")
        try:
            self.wait_text("Precland target not found. Proceeding to next WP.", timeout=200)
        except AutoTestTimeoutException as e:
            return

        self.wait_location(self.homeloc,
                           accuracy=120,
                           target_altitude=self.homeloc.alt,
                           height_accuracy=100,
                           timeout=180)

        # wait for landing to be completed
        self.mav.motors_disarmed_wait()

    def pl_proceed_gps(self):

        print("PRECLAND TEST: Target not found. Proceed on GPS.")

        # Simulated target enable
        self.set_parameter("SIM_PLD_ENABLE", 0)
        #self.load_mission("swoop-precland-mission.txt")
        self.pl_start_mission("quadplane_precland_proceed_gps_mission.txt")
        
        # wait for landing to be completed
        self.mav.motors_disarmed_wait()

        # check that it didn't land on the target
        self.assert_not_visual_landing()

        # return to start
        self.pl_return()
        

    def pl_abort_contingency(self):

        print("PRECLAND TEST: Target not found. Abort, proceed to contingency site.")


         # Simulated target enable
        self.set_parameter("SIM_PLD_ENABLE", 0)
        #self.load_mission("swoop-precland-mission.txt")
        self.pl_start_mission("quadplane_precland_abort_contingency_mission.txt")
        try:
            self.wait_text("Precland target not found. Proceeding to contingency WP.", timeout=200)
        except AutoTestTimeoutException as e:
            return
        
         # wait for landing to be completed
        self.mav.motors_disarmed_wait()

        # return to start
        self.pl_return()

    def pl_abort_next_wp_lose_target_above_abort_alt(self):
        print("PRECLAND TEST: Target lost above abort alt. Abort, continue to next WP.")


        # Simulated target enable
        self.set_parameter("SIM_PLD_ENABLE", 1)
        #self.load_mission("swoop-precland-mission.txt")
        self.pl_start_mission("quadplane_precland_abort_next_wp_mission.txt")

        # Turn off target above abort altitude
        # Assumes default abort alt of 15m
        self.wait_altitude(20, 30, timeout=300, relative=True)
        self.set_parameter("SIM_PLD_ENABLE", 0)
        
        # Ensure it turned around and went back home
        self.wait_location(self.homeloc,
                           accuracy=120,
                           target_altitude=self.homeloc.alt,
                           height_accuracy=100,
                           timeout=300)

        # wait for landing to be completed
        self.mav.motors_disarmed_wait()
        
    def pl_abort_next_wp_lose_target_below_abort_alt(self):
        print("PRECLAND TEST: Target lost below abort alt. Proceed with visual landing")

        # Simulated target enable
        self.set_parameter("SIM_PLD_ENABLE", 1)
        #self.load_mission("swoop-precland-mission.txt")
        self.pl_start_mission("quadplane_precland_abort_next_wp_mission.txt")

        # Turn off target above abort altitude
        # Assumes default abort alt of 15m
        self.wait_altitude(12, 14, timeout=300, relative=True)
        self.set_parameter("SIM_PLD_ENABLE", 0) 
        self.mav.motors_disarmed_wait()
        self.assert_visual_landing()
        
    def pl_proceed_gps_lose_target_above_abort_alt(self):

        print("PRECLAND TEST: Target lost above abort alt. Proceed on GPS.")

        # Simulated target enable
        self.set_parameter("SIM_PLD_ENABLE", 1)
        
        self.pl_start_mission("quadplane_precland_proceed_gps_mission.txt")

        # Turn off target above abort altitude
        # Assumes default abort alt of 15m
        self.wait_altitude(20, 30, timeout=300, relative=True)
        self.set_parameter("SIM_PLD_ENABLE", 0) 
        
        # wait for landing to be completed
        self.mav.motors_disarmed_wait()

        # check that it landed on the target
        self.assert_visual_landing()

        # return to start
        self.pl_return()

    def pl_proceed_gps_lose_target_below_abort_alt(self):

        print("PRECLAND TEST: Target lost below abort alt. Continue with Visual landing.")

        # Simulated target enable
        self.set_parameter("SIM_PLD_ENABLE", 1)
        
        self.pl_start_mission("quadplane_precland_proceed_gps_mission.txt")

        # Turn off target above abort altitude
        # Assumes default abort alt of 15m
        self.wait_altitude(12, 14, timeout=300, relative=True)
        self.set_parameter("SIM_PLD_ENABLE", 0) 
        
        # wait for landing to be completed
        self.mav.motors_disarmed_wait()

        # check that it landed on the target
        self.assert_visual_landing()

        # return to start
        self.pl_return()

    def assert_visual_landing(self):
        dist = self.get_dist_to_pl_target()
        self.progress("Visual Landing Error: {}m".format(dist))
        if dist > 2: #m
            raise NotAchievedException("Vehicle location not near visual landing target ({}m > {}m)".format(dist, 2))
    
    def assert_not_visual_landing(self):
        dist = self.get_dist_to_pl_target()
        self.progress("Visual Landing Error: {}m".format(dist))
        if dist < 2: #m
            raise NotAchievedException("Vehicle location near visual landing target when not supposed to be ({}m < {}m)".format(dist, 2))


    def get_dist_to_pl_target(self):
        # get_distance doesn't seem to be working all that well, so do it manually

        t_lat = -27.2721798
        t_lon = 151.30387505

        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        x = mavutil.location(m.lat/1e7, m.lon/1e7, m.alt/1e3, 0)
        v_lat = x.lat
        v_lon = x.lng

        d_lat = t_lat - v_lat
        d_lon = t_lon - v_lon

        dist = math.sqrt((d_lat*d_lat)+(d_lon*d_lon*math.cos(v_lat * (math.radians(1)))))*1.113195e5
        return dist


    def default_mode(self):
        return "MANUAL"

    def disabled_tests(self):
        return {
            "QAutoTune": "See https://github.com/ArduPilot/ardupilot/issues/10411",
        }

    def tests(self):
        '''return list of all tests'''
        m = os.path.join(testdir, "ArduPlane-Missions/Dalby-OBC2016.txt")
        f = os.path.join(testdir,
                         "ArduPlane-Missions/Dalby-OBC2016-fence.txt")

        ret = super(AutoTestQuadPlane, self).tests()
        ret.extend([
            ("TestMotorMask", "Test output_motor_mask", self.test_motor_mask),

            ("TestPrecisionLanding", "Test Precision Landing", self.test_precision_landing),

            ("ParameterChecks",
             "Test Arming Parameter Checks",
             self.test_parameter_checks),

            ("Mission", "Dalby Mission",
             lambda: self.fly_mission(m, f))
        ])
        return ret
