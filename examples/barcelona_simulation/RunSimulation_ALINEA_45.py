# #############################################################################
# ####### EQ-ALINEA - FAIRNESS-ENHANCING  RAMP METERING FOR METROPOLIAN HIGHWAYS
# #######
# #######     AUTHOR:       Kevin Riehl <kriehl@ethz.ch>, Yangle Zhan <zhanyang@student.ethz.ch>
# #######     YEAR :        2025
# #######     ORGANIZATION: Traffic Engineering Group (SVT),
# #######                   Institute for Transportation Planning and Systems,
# #######                   ETH Zürich
# #############################################################################
"""
This code will run a microsimulation with the EQ-ALINEA ramp metering controller
and generate relevant log files. There are certain options to run the simulation,
such as uncontroller scenario, Donwstream ALINEA, Upstream ALINEA, and the same
for EQ-ALINEA.
"""


# #############################################################################
# ## IMPORTS
# #############################################################################
import os
import sys

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci
import numpy as np
from collections import defaultdict
import math


# #############################################################################
# ## RUN ARGUMENTS & PARSING
# #############################################################################
# def printHelpStatement():
#     print("============================================================")
#     print("SUMO MICROSIMULATION \"EQ-ALINEA - FAIRNESS-ENHANCING  RAMP METERING FOR METROPOLIAN HIGHWAYS\"")
#     print("by Kevin Riehl and Yangle Zhan 2025 (IVT, ETH Zürich) <kriehl@ethz.ch>, <zhanyang@student.ethz.ch>")
#     print("============================================================")
#     print("This code will run a microsimulation with the EQ-ALINEA ramp metering controller\n and generate relevant log files.")
#     print("============================================================")
#     print("Usage: python RunSimulation.py --sumo-path [A] --controller [B]")
#     print("\t[A] path to SUMO installation directory")
#     print("\t[B] control algorithm,\n\tOptions: \"NO_CONTROL\", \"DOWN_ALINEA\", \"UP_ALINEA\", \"DOWN_EQ_ALINEA\", \"UP_EQ_ALINEA\"")
#     print("============================================================")
# args = sys.argv
# if "help" or "--h" or "--help" in args:
#     printHelpStatement()
#     sys.exit(0)
# if len(args)>=3:
#     if args[1]=="--sumo-path":
#         sumoBinary = args[2]
#     else:
#         print("WRONG INPUT")
#         printHelpStatement()
#         sys.exit(0)
# else:
#     printHelpStatement()
#     sys.exit(0)
# if len(args)>=5:
#     if args[3]=="--controller":
#         CONTROL_MODE = args[4]
#         if not CONTROL_MODE in ["NO_CONTROL", "DOWN_ALINEA", "UP_ALINEA", "DOWN_EQ_ALINEA", "UP_EQ_ALINEA"]:
#             print("WRONG controller!")
#             printHelpStatement()
#             sys.exit(0)
# if DEBUG_GUI:
sumoBinary = "C:/Users/kriehl/AppData/Local/sumo-1.19.0/bin/sumo-gui.exe"
# else:
#     sumoBinary = "C:/Users/kriehl/AppData/Local/sumo-1.19.0/bin/sumo.exe"
CONTROL_MODE = "UP_ALINEA"


# #############################################################################
# ## PARAMETERS
# #############################################################################

# TIME RELEVANT
STEPS_PER_SECOND = 1  # steps/sec
SIMULATION_DURATION = 10800  # seconds = 3 hours
SIGNAL_CYCLE_DURATION = 45  # sec
RECORDING_CONTROL_STATS_START_TIME = 240.0
VEHICLE_AV_ACC_TIME = 2.0  # sec/veh
# SIMULATION _RELEVANT
SPAWN_CAR_PERCENTAGE = 0.9  # rate
SPAWN_MOTORCYCLE_PERCENTAGE = 0.1  # rate
RAMP_SENSOR_MATCHING = {
    k: [f"e2_E{k[2:]}_{i}" for i in range(8)]
    for k in ["JE2", "JE3", "JE4", "JE4b", "JE5"]
    if k.startswith("JE")
}
SUMO_CONFIG_FILE = "Configuration.sumocfg"
# CONTROL RELEVANT
K_R = 70 / 60  # veh/min
OCCUPANCY_TARGET = 20.0  # target occupancy, 20 %
STATE_MEMORY_HORIZON = 10  # last 10 seconds
TAU = 2.5  # m
Q_MIN = 0  # veh/h
Q_MAX = 1800  # veh/h
MU = 0.8
R_B = 0.5
R_C = 0.3
AV_CAR_LENGTH = 5
AV_MOTORCYCLE_LENGTH = 2.2
VEHICLE_AV_LENGTH = (
    SPAWN_MOTORCYCLE_PERCENTAGE * AV_MOTORCYCLE_LENGTH
    + SPAWN_CAR_PERCENTAGE * AV_CAR_LENGTH
)


# #############################################################################
# ## CONTROL METHODS
# #############################################################################
def execute_ramp_metering_control(ramp, control_mode, current_time):
    if current_time > RECORDING_CONTROL_STATS_START_TIME:
        if (
            current_time % SIGNAL_CYCLE_DURATION == 0
        ):  # every SIGNAL_CYCLE_DURATION seconds
            # determine new metering rates
            if control_mode == "NO_CONTROL":
                return
            elif control_mode == "DOWN_ALINEA":
                q_rate[ramp], metering_rate[ramp] = control_ALINEA(
                    ramp=ramp,
                    q_previous_rate=traffic_data[ramp]["real_Q_flow"][-2],
                    occupancy_measured=traffic_data[ramp]["Downstream_O_Measured"][-1],
                )
            elif control_mode == "UP_ALINEA":
                q_rate[ramp], metering_rate[ramp] = control_ALINEA(
                    ramp=ramp,
                    q_previous_rate=traffic_data[ramp]["real_Q_flow"][-2],
                    occupancy_measured=traffic_data[ramp]["Upstream_O_Measured"][-1],
                )
            elif control_mode == "UP_EQ_ALINEA":
                q_rate[ramp], metering_rate[ramp] = control_EQ_ALINEA(
                    ramp=ramp,
                    q_previous_rate=traffic_data[ramp]["real_Q_flow"][
                        -2
                    ],  # Pass the whole dictionary
                    q_4_previous_rates=traffic_data[ramp]["real_Q_flow"][-2]
                    + traffic_data[ramp]["real_Q_flow"][-62]
                    + traffic_data[ramp]["real_Q_flow"][-122]
                    + traffic_data[ramp]["real_Q_flow"][-182],
                    occupancy_measured=traffic_data[ramp]["Upstream_O_Measured"][-1],
                    previous_metering_rates=traffic_data[ramp]["metering_rates"],
                    queue_length=traffic_data[ramp]["queue_lengths"][-1],
                    detector_length=traffic_data[ramp]["detector_length"],
                )
            elif control_mode == "DOWN_EQ_ALINEA":
                q_rate[ramp], metering_rate[ramp] = control_EQ_ALINEA(
                    ramp=ramp,
                    q_previous_rate=traffic_data[ramp]["real_Q_flow"][
                        -2
                    ],  # Pass the whole dictionary
                    q_4_previous_rates=traffic_data[ramp]["real_Q_flow"][-2]
                    + traffic_data[ramp]["real_Q_flow"][-62]
                    + traffic_data[ramp]["real_Q_flow"][-122]
                    + traffic_data[ramp]["real_Q_flow"][-182],
                    occupancy_measured=traffic_data[ramp]["Downstream_O_Measured"][
                        -1
                    ],  # downstream detector are used here
                    previous_metering_rates=traffic_data[ramp]["metering_rates"],
                    queue_length=traffic_data[ramp]["queue_lengths"][-1],
                    detector_length=traffic_data[ramp]["detector_length"],
                )
            # update signal control logic
            update_ramp_signal_control_logic(
                metering_rate[ramp], SIGNAL_CYCLE_DURATION, ramp
            )


def control_ALINEA(ramp, q_previous_rate, occupancy_measured):
    """
    ALINEA control logic for a single ramp.

    Parameters:
    - ramp: The intersection ID.
    - q_previous_rate: Dictionary of previous flow rates for ramps.
    - occupancy_measured: Measured occupancy.
    Returns:
    - q_rate: The updated flow rate for the ramp.
    - metering_rate: The calculated green share for the ramp. (%)
    """
    q_rate = q_previous_rate + K_R * (OCCUPANCY_TARGET - occupancy_measured)
    q_bounded = min(Q_MAX, max(q_rate, Q_MIN))
    metering_rate = (q_bounded * VEHICLE_AV_ACC_TIME) / SIGNAL_CYCLE_DURATION
    metering_rate = min(1.0, math.floor(metering_rate * 10) / 10)
    return q_rate, metering_rate


def control_EQ_ALINEA(
    ramp,
    q_previous_rate,
    q_4_previous_rates,
    occupancy_measured,
    previous_metering_rates,
    queue_length,
    detector_length,
):
    """
    ALINEA control logic for a single ramp.

    Parameters:
    - ramp: The intersection ID.
    - q_previous_rate: Dictionary of previous flow rates for ramps.
    - occupancy_measured: Measured occupancy at the exit.
    - previous_metering_rates: List of previous green share values.
    - queue_length: Current queue length.
    - detector_length: Length of the detector (for jam assessment).
    Returns:
    - q_rate: The updated flow rate for the ramp.
    - metering_rate: The calculated green share for the ramp. (%)
    """
    q_rate = q_previous_rate + K_R * (OCCUPANCY_TARGET - occupancy_measured)
    q_bounded = min(Q_MAX, max(q_rate, Q_MIN))
    q_tau = detector_length / (VEHICLE_AV_LENGTH + TAU)
    # ALINEA CASE
    if (q_bounded > 0.0) and (q_4_previous_rates > q_tau):
        metering_rate = (q_bounded * VEHICLE_AV_ACC_TIME) / SIGNAL_CYCLE_DURATION
        metering_rate = min(1.0, math.floor(metering_rate * 10) / 10)
    # BOUNDARY CONDITION I: Sufficient time for on-ramp dequeuing
    elif (q_bounded > 0.0) and (q_4_previous_rates <= q_tau):
        metering_rate = (
            (q_tau - q_4_previous_rates) * VEHICLE_AV_ACC_TIME / SIGNAL_CYCLE_DURATION
        )
        metering_rate = min(1.0, math.floor(metering_rate * 10) / 10)
    # BOUNDARY CONDITION II: Maximum waiting times
    elif len(previous_metering_rates) >= 180 and all(
        value == 0 for value in previous_metering_rates[-180:]
    ):  # 240==4cycle*60*1(step)
        metering_rate = R_B
    # BOUNDARY CONDITION III: Queue-Spill-Back Prevention
    elif queue_length >= (MU * detector_length):
        metering_rate = R_C
    # ELSE CASE
    else:
        metering_rate = 0.0
    return q_rate, metering_rate


def update_ramp_signal_control_logic(metering_rate, cycle_duration, ramp):
    """
    Adjusts traffic light phases based on the green time share.

    Parameters:
    - metering_rate: proportion of green time in the cycle (0.0 - 1.0)
    - cycle_duration: total duration of one cycle
    - ramp: traffic light intersection ID (from SUMO)
    """
    # determine red and green time
    green_time = int(metering_rate * cycle_duration)
    red_time = cycle_duration - green_time
    # write new times to traffic light control logic
    traffic_light_logic = traci.trafficlight.getAllProgramLogics(ramp)[0]
    for ph_id in range(0, len(traffic_light_logic.phases)):
        if ph_id == 0:  # G
            traffic_light_logic.phases[ph_id].minDur = green_time
            traffic_light_logic.phases[ph_id].maxDur = green_time
            traffic_light_logic.phases[ph_id].duration = green_time
        elif ph_id == 1:  # R
            traffic_light_logic.phases[ph_id].minDur = red_time
            traffic_light_logic.phases[ph_id].maxDur = red_time
            traffic_light_logic.phases[ph_id].duration = red_time
    traci.trafficlight.setProgramLogic(ramp, traffic_light_logic)
    # reset traffic controller to use new logic
    traci.trafficlight.setPhase(ramp, 0)


# #############################################################################
# ## RECORDING METHODS
# #############################################################################


def generateInitialVehicleData():
    vehicles_data = defaultdict(
        lambda: {
            "VehicleID": None,  # ID of the vehicle
            # Time when the vehicle entered the network (in seconds)
            "EntryTime": np.nan,
            # Time when the vehicle exited the network (in seconds)
            "ExitTime": np.nan,
            # Total time the vehicle spent in the network (in seconds)
            "TravelTime": np.nan,
            # Total distance traveled by the vehicle (in meters)
            "Vehicles_Travel_Distance": np.nan,
        }
    )
    return vehicles_data


def generateInitialTrafficData():
    traffic_data = {}
    for ramp in RAMP_SENSOR_MATCHING:
        traffic_data[ramp] = {
            "simulation_times": [],
            "average_speeds": [],
            "metering_rates": [],
            "queue_lengths": [],
            "real_signals": [],
            "Upstream_average_speeds": [],
            "Upstream_speed_memory": [],
            "Upstream_Interval_average_speeds": [],
            "Upstream_Occup_Left": [],
            "Upstream_Occup_Middle": [],
            "Upstream_Occup_Right": [],
            "Upstream_occupancy_memory": [],
            "Upstream_O_Measured": [],
            "Upstream_vehNL": [],
            "Upstream_vehNM": [],
            "Upstream_vehNR": [],
            "Upstream_average_flow": [],
            "Upstream_Interval_O_Measured": [],
            "Upstream_Interval_O_L": [],
            "Upstream_Interval_O_M": [],
            "Upstream_Interval_O_R": [],
            "Upstream_Interval_vehNL": [],
            "Upstream_Interval_vehNM": [],
            "Upstream_Interval_vehNR": [],
            "Upstream_Interval_average_flow": [],
            "Downstream_average_speeds": [],
            "Downstream_speed_memory": [],
            "Downstream_Interval_average_speeds": [],
            "Downstream_Occup_Left": [],
            "Downstream_Occup_Middle": [],
            "Downstream_Occup_Right": [],
            "Downstream_occupancy_memory": [],
            "Downstream_O_Measured": [],
            "Downstream_vehNL": [],
            "Downstream_vehNM": [],
            "Downstream_vehNR": [],
            "Downstream_average_flow": [],
            "Downstream_Interval_O_Measured": [],
            "Downstream_Interval_O_L": [],
            "Downstream_Interval_O_M": [],
            "Downstream_Interval_O_R": [],
            "Downstream_Interval_vehNL": [],
            "Downstream_Interval_vehNM": [],
            "Downstream_Interval_vehNR": [],
            "Downstream_Interval_average_flow": [],
            "Q_rate": [],
            "real_Q_flow": [],
            "detector_length": float("nan"),
            "TL_state": [],
        }
    return traffic_data


def record_vehicle_data(step, vehicles_data):
    """
    Updates the vehicle data dictionary during the simulation.

    Parameters:
    - step: Current time step in the simulation.
    """
    # Vehicles that have just entered the network
    departed_vehicles = traci.simulation.getDepartedIDList()
    for vehicle in departed_vehicles:
        # Record the vehicle ID    Redundant
        vehicles_data[vehicle]["VehicleID"] = vehicle
        # Record the entry time (convert from steps to seconds)
        vehicles_data[vehicle]["EntryTime"] = step / STEPS_PER_SECOND

    # Vehicles currently active in the network
    active_vehicles = traci.vehicle.getIDList()
    for vehicle in active_vehicles:
        # Get the current distance traveled by the vehicle
        traveled_distance = traci.vehicle.getDistance(vehicle)
        # Update the travel distance
        vehicles_data[vehicle]["Vehicles_Travel_Distance"] = traveled_distance

    # Vehicles that have exited the network
    arrived_vehicles = traci.simulation.getArrivedIDList()
    for vehicle in arrived_vehicles:
        # Ensure the entry time is recorded
        if vehicles_data[vehicle]["EntryTime"] is not np.nan:
            vehicles_data[vehicle]["ExitTime"] = (
                step / STEPS_PER_SECOND
            )  # Record the exit time
            vehicles_data[vehicle][
                "TravelTime"
            ] = (  # Calculate travel time as the difference between exit and entry times
                vehicles_data[vehicle]["ExitTime"] - vehicles_data[vehicle]["EntryTime"]
            )


def record_upstream_traffic_data(ramp):
    # Upstream Occupancy
    Upstream_occupL = traci.lanearea.getLastStepOccupancy(detectors[7])
    Upstream_occupM = traci.lanearea.getLastStepOccupancy(detectors[6])
    Upstream_occupR = traci.lanearea.getLastStepOccupancy(detectors[5])
    Upstream_weightO = (
        1 / 3 * Upstream_occupL + 1 / 3 * Upstream_occupM + 1 / 3 * Upstream_occupR
    )  # Normalize to [0-1]
    # Test for Interval Occupancy
    Upstream_Interval_O_L = traci.lanearea.getIntervalOccupancy(detectors[7])
    Upstream_Interval_O_M = traci.lanearea.getIntervalOccupancy(detectors[6])
    Upstream_Interval_O_R = traci.lanearea.getIntervalOccupancy(detectors[5])
    Upstream_Interval_O_Measured = (
        1 / 3 * Upstream_Interval_O_L
        + 1 / 3 * Upstream_Interval_O_M
        + 1 / 3 * Upstream_Interval_O_R
    )
    traffic_data[ramp]["Upstream_Interval_O_L"].append(Upstream_Interval_O_L)
    traffic_data[ramp]["Upstream_Interval_O_M"].append(Upstream_Interval_O_M)
    traffic_data[ramp]["Upstream_Interval_O_R"].append(Upstream_Interval_O_R)
    traffic_data[ramp]["Upstream_Interval_O_Measured"].append(
        Upstream_Interval_O_Measured
    )
    # Traffic_data storage
    traffic_data[ramp]["Upstream_Occup_Left"].append(Upstream_occupL)
    traffic_data[ramp]["Upstream_Occup_Middle"].append(Upstream_occupM)
    traffic_data[ramp]["Upstream_Occup_Right"].append(Upstream_occupR)
    traffic_data[ramp]["Upstream_occupancy_memory"].append(Upstream_weightO)
    # Rolling window for average occupancy
    if len(traffic_data[ramp]["Upstream_occupancy_memory"]) > STATE_MEMORY_HORIZON:
        # delete first element
        traffic_data[ramp]["Upstream_occupancy_memory"] = traffic_data[ramp][
            "Upstream_occupancy_memory"
        ][1:]
    Upstream_O_Measured = np.mean(traffic_data[ramp]["Upstream_occupancy_memory"])
    traffic_data[ramp]["Upstream_O_Measured"].append(Upstream_O_Measured)
    # Upstream Speeds
    Upstream_speedL = traci.lanearea.getLastStepMeanSpeed(detectors[7])
    Upstream_speedM = traci.lanearea.getLastStepMeanSpeed(detectors[6])
    Upstream_speedR = traci.lanearea.getLastStepMeanSpeed(detectors[5])
    Upstream_vehNL = traci.lanearea.getLastStepVehicleNumber(detectors[7])
    Upstream_vehNM = traci.lanearea.getLastStepVehicleNumber(detectors[6])
    Upstream_vehNR = traci.lanearea.getLastStepVehicleNumber(detectors[5])
    Upstream_Interval_vehNL = traci.lanearea.getLastIntervalVehicleNumber(detectors[7])
    Upstream_Interval_vehNM = traci.lanearea.getLastIntervalVehicleNumber(detectors[6])
    Upstream_Interval_vehNR = traci.lanearea.getLastIntervalVehicleNumber(detectors[5])
    # Upstream average speeds
    if (Upstream_vehNL + Upstream_vehNM + Upstream_vehNR) > 0:
        Upstream_weightS = (
            Upstream_speedL * Upstream_vehNL
            + Upstream_speedM * Upstream_vehNM
            + Upstream_speedR * Upstream_vehNR
        ) / (Upstream_vehNL + Upstream_vehNM + Upstream_vehNR)
        traffic_data[ramp]["Upstream_speed_memory"].append(Upstream_weightS)
        if len(traffic_data[ramp]["Upstream_speed_memory"]) > STATE_MEMORY_HORIZON:
            # delete first element
            traffic_data[ramp]["Upstream_speed_memory"] = traffic_data[ramp][
                "Upstream_speed_memory"
            ][1:]
    Upstream_average_speed = np.mean(traffic_data[ramp]["Upstream_speed_memory"])
    traffic_data[ramp]["Upstream_average_speeds"].append(Upstream_average_speed)
    # UpstreamIntervalSpeed
    Upstream_Interval_Speed_L = traci.lanearea.getLastIntervalMeanSpeed(detectors[7])
    Upstream_Interval_Speed_M = traci.lanearea.getLastIntervalMeanSpeed(detectors[6])
    Upstream_Interval_Speed_R = traci.lanearea.getLastIntervalMeanSpeed(detectors[5])
    traffic_data[ramp]["Upstream_Interval_average_speeds"].append(
        (
            Upstream_Interval_Speed_L
            + Upstream_Interval_Speed_M
            + Upstream_Interval_Speed_R
        )
        / 3
    )
    # Compute Entry flow
    Upstream_average_flow = (
        (Upstream_vehNL + Upstream_vehNM + Upstream_vehNR) / 3 * (3600)
    )
    traffic_data[ramp]["Upstream_average_flow"].append(Upstream_average_flow)
    traffic_data[ramp]["Upstream_vehNL"].append(Upstream_vehNL)
    traffic_data[ramp]["Upstream_vehNM"].append(Upstream_vehNM)
    traffic_data[ramp]["Upstream_vehNR"].append(Upstream_vehNR)
    Upstream_Interval_average_flow = (
        (Upstream_Interval_vehNL + Upstream_Interval_vehNM + Upstream_Interval_vehNR)
        / (3 * 10)
        * (3600)
    )  # 10 is the detector period
    traffic_data[ramp]["Upstream_Interval_average_flow"].append(
        Upstream_Interval_average_flow
    )
    traffic_data[ramp]["Upstream_Interval_vehNL"].append(Upstream_Interval_vehNL)
    traffic_data[ramp]["Upstream_Interval_vehNM"].append(Upstream_Interval_vehNM)
    traffic_data[ramp]["Upstream_Interval_vehNR"].append(Upstream_Interval_vehNR)


def record_downstream_traffic_data(ramp):
    # Downstream Occupancy
    Downstream_occupL = traci.lanearea.getLastStepOccupancy(detectors[4])
    Downstream_occupM = traci.lanearea.getLastStepOccupancy(detectors[3])
    Downstream_occupR = traci.lanearea.getLastStepOccupancy(detectors[2])
    Downstream_weightO = (
        1 / 3 * Downstream_occupL
        + 1 / 3 * Downstream_occupM
        + 1 / 3 * Downstream_occupR
    )  # Normalize to [0-1]
    # Test for Interval Occupancy
    Downstream_Interval_O_L = traci.lanearea.getIntervalOccupancy(detectors[4])
    Downstream_Interval_O_M = traci.lanearea.getIntervalOccupancy(detectors[3])
    Downstream_Interval_O_R = traci.lanearea.getIntervalOccupancy(detectors[2])
    Downstream_Interval_O_Measured = (
        1 / 3 * Downstream_Interval_O_L
        + 1 / 3 * Downstream_Interval_O_M
        + 1 / 3 * Downstream_Interval_O_R
    )
    traffic_data[ramp]["Downstream_Interval_O_L"].append(Downstream_Interval_O_L)
    traffic_data[ramp]["Downstream_Interval_O_M"].append(Downstream_Interval_O_M)
    traffic_data[ramp]["Downstream_Interval_O_R"].append(Downstream_Interval_O_R)
    traffic_data[ramp]["Downstream_Interval_O_Measured"].append(
        Downstream_Interval_O_Measured
    )
    traffic_data[ramp]["Downstream_Occup_Left"].append(Downstream_occupL)
    traffic_data[ramp]["Downstream_Occup_Middle"].append(Downstream_occupM)
    traffic_data[ramp]["Downstream_Occup_Right"].append(Downstream_occupR)
    traffic_data[ramp]["Downstream_occupancy_memory"].append(Downstream_weightO)
    # Rolling window for average occupancy
    if len(traffic_data[ramp]["Downstream_occupancy_memory"]) > STATE_MEMORY_HORIZON:
        # delete first element
        traffic_data[ramp]["Downstream_occupancy_memory"] = traffic_data[ramp][
            "Downstream_occupancy_memory"
        ][1:]
    Downstream_O_Measured = np.mean(traffic_data[ramp]["Downstream_occupancy_memory"])
    traffic_data[ramp]["Downstream_O_Measured"].append(Downstream_O_Measured)
    # Downstream speeds
    Downstream_speedL = traci.lanearea.getLastStepMeanSpeed(detectors[4])
    Downstream_speedM = traci.lanearea.getLastStepMeanSpeed(detectors[3])
    Downstream_speedR = traci.lanearea.getLastStepMeanSpeed(detectors[2])
    Downstream_vehNL = traci.lanearea.getLastStepVehicleNumber(detectors[4])
    Downstream_vehNM = traci.lanearea.getLastStepVehicleNumber(detectors[3])
    Downstream_vehNR = traci.lanearea.getLastStepVehicleNumber(detectors[2])
    Downstream_Interval_vehNL = traci.lanearea.getLastIntervalVehicleNumber(
        detectors[4]
    )
    Downstream_Interval_vehNM = traci.lanearea.getLastIntervalVehicleNumber(
        detectors[3]
    )
    Downstream_Interval_vehNR = traci.lanearea.getLastIntervalVehicleNumber(
        detectors[2]
    )
    if (Downstream_vehNL + Downstream_vehNM + Downstream_vehNR) > 0:
        Downstream_weightS = (
            Downstream_speedL * Downstream_vehNL
            + Downstream_speedM * Downstream_vehNM
            + Downstream_speedR * Downstream_vehNR
        ) / (Downstream_vehNL + Downstream_vehNM + Downstream_vehNR)
        traffic_data[ramp]["Downstream_speed_memory"].append(Downstream_weightS)
        if len(traffic_data[ramp]["Downstream_speed_memory"]) > STATE_MEMORY_HORIZON:
            # delete first element
            traffic_data[ramp]["Downstream_speed_memory"] = traffic_data[ramp][
                "Downstream_speed_memory"
            ][1:]
    Downstream_average_speed = np.mean(traffic_data[ramp]["Downstream_speed_memory"])
    traffic_data[ramp]["Downstream_average_speeds"].append(Downstream_average_speed)
    # DownstreamIntervalSpeed
    Downstream_Interval_Speed_L = traci.lanearea.getLastIntervalMeanSpeed(detectors[4])
    Downstream_Interval_Speed_M = traci.lanearea.getLastIntervalMeanSpeed(detectors[3])
    Downstream_Interval_Speed_R = traci.lanearea.getLastIntervalMeanSpeed(detectors[2])
    traffic_data[ramp]["Downstream_Interval_average_speeds"].append(
        (
            Downstream_Interval_Speed_L
            + Downstream_Interval_Speed_M
            + Downstream_Interval_Speed_R
        )
        / 3
    )
    # Downstream Flow
    Downstream_average_flow = (
        (Downstream_vehNL + Downstream_vehNM + Downstream_vehNR) / 3 * (3600)
    )
    traffic_data[ramp]["Downstream_average_flow"].append(Downstream_average_flow)
    traffic_data[ramp]["Downstream_vehNL"].append(Downstream_vehNL)
    traffic_data[ramp]["Downstream_vehNM"].append(Downstream_vehNM)
    traffic_data[ramp]["Downstream_vehNR"].append(Downstream_vehNR)
    Downstream_Interval_average_flow = (
        (
            Downstream_Interval_vehNL
            + Downstream_Interval_vehNM
            + Downstream_Interval_vehNR
        )
        / (3 * 10)
        * (3600)
    )  # 10 is the detector period
    traffic_data[ramp]["Downstream_Interval_average_flow"].append(
        Downstream_Interval_average_flow
    )
    traffic_data[ramp]["Downstream_Interval_vehNL"].append(Downstream_Interval_vehNL)
    traffic_data[ramp]["Downstream_Interval_vehNM"].append(Downstream_Interval_vehNM)
    traffic_data[ramp]["Downstream_Interval_vehNR"].append(Downstream_Interval_vehNR)


def record_ramp_traffic_data(ramp):
    # Determine On-Ramp data
    queue_length = traci.lanearea.getJamLengthMeters(detectors[0])
    traffic_data[ramp]["queue_lengths"].append(queue_length)
    # Get on-ramp Q-flow
    real_Q_flow[ramp] = float(traci.lanearea.getLastIntervalVehicleNumber(detectors[1]))
    traffic_data[ramp]["real_Q_flow"].append(real_Q_flow[ramp])


def record_signal_traffic_data(ramp):
    current_state[ramp] = traci.trafficlight.getRedYellowGreenState(ramp)
    traffic_data[ramp]["TL_state"].append(current_state[ramp])
    if current_state[ramp] == "r":
        traffic_data[ramp]["real_signals"].append(0)
    else:  # "g"
        traffic_data[ramp]["real_signals"].append(100)
    traffic_data[ramp]["metering_rates"].append(metering_rate[ramp])
    traffic_data[ramp]["Q_rate"].append(q_rate[ramp])


# #############################################################################
# ## MAIN CODE
# #############################################################################

# LAUNCH SUMO
sumoCmd = [
    sumoBinary,
    "-c",
    SUMO_CONFIG_FILE,
    "--start",
    "--quit-on-end",
    "--time-to-teleport",
    "-1",
]
traci.start(sumoCmd)

# INITIALIZE DATA STORAGE / RECORDER
vehicles_data = generateInitialVehicleData()
traffic_data = generateInitialTrafficData()
real_Q_flow = {ramp: 0.00 for ramp in RAMP_SENSOR_MATCHING}
q_rate = {ramp: 0.00 for ramp in RAMP_SENSOR_MATCHING}
current_state = {ramp: "" for ramp in RAMP_SENSOR_MATCHING}
metering_rate = {ramp: 1.00 for ramp in RAMP_SENSOR_MATCHING}

# INITIALIZE CONFIGURATION OF TRAFFIC LIGHT CONTROLLERS
for ramp in RAMP_SENSOR_MATCHING:
    update_ramp_signal_control_logic(metering_rate[ramp], SIGNAL_CYCLE_DURATION, ramp)
    traffic_data[ramp]["detector_length"] = traci.lanearea.getLength(
        RAMP_SENSOR_MATCHING[ramp][0]
    )


# 06:45-07:15
VEH_POS_LOG_START = int(3600 * 0.75)  # int(3600*2.5)
VEH_POS_LOG_END = int(3600 * 1.25)  # int(3600*3)

LOG_FILE_1 = "xx_vehicle_pos_log_file_" + CONTROL_MODE + ".csv"
LOG_FILE_2 = "xx_vehicle_pos_log_file_" + CONTROL_MODE + "_lights.csv"
SEPARATOR = ","

fW = open(LOG_FILE_1, "a+")
fW2 = open(LOG_FILE_2, "a+")

fW.write(
    "time"
    + SEPARATOR
    + "veh_id"
    + SEPARATOR
    + "pos_x"
    + SEPARATOR
    + "pos_y"
    + SEPARATOR
    + "angle"
)
fW.write("\n")

fW2.write("time" + SEPARATOR + "state")
fW2.write("\n")

# RUN SIMULATION
for step in range(0, SIMULATION_DURATION * STEPS_PER_SECOND):
    # Execute Simulation Step
    record_vehicle_data(step, vehicles_data)

    # Record Traffic Data
    for ramp in RAMP_SENSOR_MATCHING:
        detectors = RAMP_SENSOR_MATCHING[ramp][:]
        record_upstream_traffic_data(ramp)
        record_downstream_traffic_data(ramp)
        record_ramp_traffic_data(ramp)
        traffic_data[ramp]["simulation_times"].append(traci.simulation.getTime())

    if step % 100 == 0:
        print(step)

    # Execute Ramp Metering Control
    current_time = step / STEPS_PER_SECOND
    for ramp in RAMP_SENSOR_MATCHING:
        detectors = RAMP_SENSOR_MATCHING[ramp][:]
        execute_ramp_metering_control(ramp, CONTROL_MODE, current_time)

    # LOG FOR SHIMAA
    if current_time >= VEH_POS_LOG_START and current_time <= VEH_POS_LOG_END:
        vehicle_ids = traci.vehicle.getIDList()
        for veh_id in vehicle_ids:
            pos_x, pos_y = traci.vehicle.getPosition(veh_id)
            angle = traci.vehicle.getAngle(veh_id)

            fW.write(str(current_time))
            fW.write(SEPARATOR)
            fW.write(str(veh_id))
            fW.write(SEPARATOR)
            fW.write(f"{pos_x:.4f}")
            fW.write(SEPARATOR)
            fW.write(f"{pos_y:.4f}")
            fW.write(SEPARATOR)
            fW.write(f"{angle:.4f}")
            fW.write("\n")

        fW2.write(str(current_time))
        fW2.write(SEPARATOR)
        fW2.write(str(traci.trafficlight.getRedYellowGreenState("JE3")))
        fW2.write("\n")

    if current_time >= VEH_POS_LOG_END:
        break

    # Execute Simulation Step
    traci.simulationStep()

    # Record Traffic Data - Signals
    for ramp in RAMP_SENSOR_MATCHING:
        record_signal_traffic_data(ramp)

# CLOSE SUMO
traci.close()

fW.close()
fW2.close()
