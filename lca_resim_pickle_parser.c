/****************** Includes ******************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <LCA_Core_and_StateMachine.h>
#include <pickle/lca_resim_pickle_parser.h>
#include <common/lca_sil_log.h>
/* Add python support */
#define NO_IMPORT_ARRAY
#define PY_ARRAY_UNIQUE_SYMBOL lca_sil_pickle_python_ARRAY_API
#include <common/lca_sil_python.h>

#include <pickle/import_pickle_versions_cfg.h>
#include <pickle/lca_sil_pickle_utilities.h>
#include <lca_pickle_data_access_type.h>

/****************** External Variables *******************/
/// @TODO: All coding parameters are read as zeros now since they are initialized with values at
///        later point of time in code. More investigation is needed to see how to read these parameters
///        with correct values. For now we only use codingAllInput.vehicleCoding.C_CAMERA_LAT_POS which is
///        initialized with 0
/// @TODO: C_DIST_FRONT_BUMPER_TO_REAR_AXLE is initialized with 3.72.
extern Coding_T codingAllInput; /// needed for road_edge_line_c0
// #define C_DIST_FRONT_BUMPER_TO_REAR_AXLE (3.72f)
extern bool g_toggle_printing_warning_messages;

/// These global variables are set to indicate
/// the dynamic data type of ME signals
/// sometimes int32 or int64
extern DATA_TYPE vision_free_space_data_type;
extern DATA_TYPE vision_fail_safe_data_type;
extern DATA_TYPE vision_object_detection_data_type;
extern DATA_TYPE vision_lane_support_data_type;
extern DATA_TYPE vision_lane_support_adv_data_type_a;
extern DATA_TYPE vision_lane_support_adv_data_type_b;
extern DATA_TYPE vision_lane_support_road_edge_data_type;
extern DATA_TYPE vision_lane_semantic_lane_data_type;
/****************** Defines *******************/

/// Number of needed pyobjects
#define NUMBER_OF_NEW_PYOBJECTS (6)
#define DELTATIMETHRESHOLD      (1000)
/********* Global Variable Declearation ********/

/// Flag to indicate pickle file does not have LateralCollisionAvoidanceQualifier
bool kLcaQualifierMissing = false;

/// Flag to toggle wrong type conversion severity
bool kLog_Error = true;

/// This is the starting time used in matlab scripts
double f64_time_offset_g = 0.0;

/// These pointers shall hold the null structures missing from pickle
/// file (Cx_STD, Decel, Crossing)
static st_lca_pickle_base_f64_t st_lca_pickle_base_f64_g = {NULL, NULL, 0., 0., 0};
static st_lca_pickle_base_s32_t st_lca_pickle_base_s32_g = {NULL, NULL, 0., 0., 0};
static st_lca_pickle_base_s64_t st_lca_pickle_base_s64_g = {NULL, NULL, 0., 0., 0};

/********* Local Function Decleration *********/
static void assign_f64_pickle_struct(st_lca_pickle_base_f64_t *const assign_to, st_lca_pickle_base_f64_t const *const assign_from);

static void assign_s32_pickle_struct(st_lca_pickle_base_s32_t *const assign_to, st_lca_pickle_base_s32_t const *const assign_from);
static void assign_s64_pickle_struct(st_lca_pickle_base_s64_t *const assign_to, st_lca_pickle_base_s64_t const *const assign_from);

static void set_global_f64_pickle_struct(unsigned int u32_length, double f64_time_min, double f64_time_max, double *p_time);

static void set_global_s32_pickle_struct(unsigned int u32_length, double f64_time_min, double f64_time_max, double *p_time);
static void set_global_s64_pickle_struct(unsigned int u32_length, double f64_time_min, double f64_time_max, double *p_time);

static PyObject *scale_data(PyObject *const p_data, DATA_TYPE data_type, double scale_factor);

static PyObject *replace_data_in_object(PyObject *const p_data, DATA_TYPE data_type, double original_data, double new_data);

static PyObject *add_data_in_object(PyObject *const p_data, DATA_TYPE data_type, double add_offset);

static bool parse_ego_motion(st_pickle_ego_motion_type_t *p_st_pickle_ego_motion, PyObject *const p_sp_2021);

static bool parse_vision_lane_support_host_lanes(st_pickle_vision_lane_support_t *p_st_vision_lane_support, PyObject *const p_spi_eyeq_core_lanes_road_edge);
static bool parse_vision_lane_support_adj_lanes(st_pickle_vision_lane_support_t *p_st_vision_lane_support, PyObject *const p_spi_eyeq_core_lanes_adjacent);

static bool parse_support_lane_host_left(st_pickle_vision_lane_support_lane_t *p_st_support_lane, PyObject *const p_spi_eyeq_core_lanes_host);
static bool parse_support_lane_host_right(st_pickle_vision_lane_support_lane_t *p_st_support_lane, PyObject *const p_spi_eyeq_core_lanes_host);

static bool parse_support_lane_adj_left(st_pickle_vision_lane_support_adjacent_lane_t *p_st_support_lane, PyObject *const p_spi_eyeq_core_lanes_adjacent);
static bool parse_support_lane_adj_right(st_pickle_vision_lane_support_adjacent_lane_t *p_st_support_lane, PyObject *const p_spi_eyeq_core_lanes_adjacent);

static bool parse_vision_lane_support_adv(st_pickle_vision_lane_support_adv_t *st_lane_support_adv, PyObject *const p_spi_eyeq_core_lanes_application);

static bool parse_vision_lane_support_semantic_lane(st_pickle_vision_lane_support_semantic_lane_t *p_st_lane_support_semantic_lane, PyObject *const p_spi_eyeq_core_semantic_lanes);

static bool parse_vision_lane_support_road_edge(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_host);

static bool parse_vision_lane_support_road_edge_left_first(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_host);
static bool parse_vision_lane_support_road_edge_right_first(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_host);
// static bool parse_vision_lane_support_road_edge_left_second(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_host);
// static bool parse_vision_lane_support_road_edge_right_second(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_host);

static bool parse_bcm_signal(st_pickle_bcm_signal_t *const p_st_bcm_signal, PyObject *const p_sp_2021);
static bool parse_srr_scw_object_signal(st_pickle_srr_scw_object_t *const p_st_srr_scw_object, PyObject *const p_sp_2021);
static bool parse_srr_lcw_object_signal(st_pickle_srr_lcw_object_t *const p_st_srr_lcw_object, PyObject *const p_sp_2021);
static bool parse_srr_tap_object_signal(st_pickle_srr_tap_object_t *const p_st_srr_tap_object, PyObject *const p_sp_2021);
static bool parse_srr_free_space_signal(st_pickle_srr_free_space_t *const p_st_srr_free_space, PyObject *const p_sp_2021);
static bool parse_frr_object_lists_signal(st_pickle_frr_object_list_t *const p_st_frr_object_list, PyObject *const p_sp_2021);
static bool parse_vision_object_detection(st_pickle_vision_object_detection_t *p_st_vision_object_detection, PyObject *const p_core_objects_protocol);
static bool parse_vision_fail_safe(st_pickle_vision_fail_safe_t *p_st_vision_fail_safe, PyObject *const p_core_failsafe_protocol);

/********* Local Function Definition **********/
static void assign_f64_pickle_struct(st_lca_pickle_base_f64_t *const assign_to, st_lca_pickle_base_f64_t const *const assign_from)
{
    assign_to->f64_time_max = assign_from->f64_time_max;
    assign_to->f64_time_min = assign_from->f64_time_min;
    assign_to->p_data = assign_from->p_data;
    assign_to->pf64_timestamp = assign_from->pf64_timestamp;
    assign_to->u32_length = assign_from->u32_length;
}

static void set_global_f64_pickle_struct(unsigned int u32_length, double f64_time_min, double f64_time_max, double *p_time)
{
    st_lca_pickle_base_f64_g.p_data = (double *)calloc(u32_length, sizeof(double));
    st_lca_pickle_base_f64_g.u32_length = u32_length;
    st_lca_pickle_base_f64_g.pf64_timestamp = p_time;
    st_lca_pickle_base_f64_g.f64_time_min = f64_time_min;
    st_lca_pickle_base_f64_g.f64_time_max = f64_time_max;
}

static void assign_s32_pickle_struct(st_lca_pickle_base_s32_t *const assign_to, st_lca_pickle_base_s32_t const *const assign_from)
{
    assign_to->f64_time_max = assign_from->f64_time_max;
    assign_to->f64_time_min = assign_from->f64_time_min;
    assign_to->p_data = assign_from->p_data;
    assign_to->pf64_timestamp = assign_from->pf64_timestamp;
    assign_to->u32_length = assign_from->u32_length;
}
static void assign_s64_pickle_struct(st_lca_pickle_base_s64_t *const assign_to, st_lca_pickle_base_s64_t const *const assign_from)
{
    assign_to->f64_time_max = assign_from->f64_time_max;
    assign_to->f64_time_min = assign_from->f64_time_min;
    assign_to->p_data = assign_from->p_data;
    assign_to->pf64_timestamp = assign_from->pf64_timestamp;
    assign_to->u32_length = assign_from->u32_length;
}

static void set_global_s32_pickle_struct(unsigned int u32_length, double f64_time_min, double f64_time_max, double *p_time)
{
    st_lca_pickle_base_s32_g.p_data = (int *)calloc(u32_length, sizeof(int));
    st_lca_pickle_base_s32_g.u32_length = u32_length;
    st_lca_pickle_base_s32_g.pf64_timestamp = p_time;
    st_lca_pickle_base_s32_g.f64_time_min = f64_time_min;
    st_lca_pickle_base_s32_g.f64_time_max = f64_time_max;
}

static void set_global_s64_pickle_struct(unsigned int u32_length, double f64_time_min, double f64_time_max, double *p_time)
{
    st_lca_pickle_base_s64_g.p_data = (long *)calloc(u32_length, sizeof(long));
    st_lca_pickle_base_s64_g.u32_length = u32_length;
    st_lca_pickle_base_s64_g.pf64_timestamp = p_time;
    st_lca_pickle_base_s64_g.f64_time_min = f64_time_min;
    st_lca_pickle_base_s64_g.f64_time_max = f64_time_max;
}

static PyObject *scale_data(PyObject *const p_data, DATA_TYPE data_type, double scale_factor)
{
    if (NULL == p_data)
    {
        return NULL;
    }
    else
    {
        size_t array_size;
        switch (data_type)
        {
        case UINT8:
            scale_factor = (char)scale_factor;
            char *p_u8data = (char *const)PyArray_DATA((PyArrayObject *)p_data);
            array_size = PyArray_SHAPE((PyArrayObject *)p_data)[0];
            for (size_t i = 0; i < array_size; ++i)
            {
                p_u8data[i] *= scale_factor;
            }
            break;
        case UINT16:
            scale_factor = (unsigned short)scale_factor;
            unsigned short *p_u16data = (unsigned short *const)PyArray_DATA((PyArrayObject *)p_data);
            array_size = PyArray_SHAPE((PyArrayObject *)p_data)[0];
            for (size_t i = 0; i < array_size; ++i)
            {
                p_u16data[i] *= scale_factor;
            }
            break;
        case INT32:
            scale_factor = (int)scale_factor;
            int *p_s32data = (int *const)PyArray_DATA((PyArrayObject *)p_data);
            array_size = PyArray_SHAPE((PyArrayObject *)p_data)[0];
            for (size_t i = 0; i < array_size; ++i)
            {
                p_s32data[i] *= scale_factor;
            }
            break;
        case INT64:
            scale_factor = (long)scale_factor;
            long *p_s64data = (long *const)PyArray_DATA((PyArrayObject *)p_data);
            array_size = PyArray_SHAPE((PyArrayObject *)p_data)[0];
            for (size_t i = 0; i < array_size; ++i)
            {
                p_s64data[i] *= scale_factor;
            }
            break;
        case FLOAT32:
            scale_factor = (float)scale_factor;
            float *p_f32data = (float *const)PyArray_DATA((PyArrayObject *)p_data);
            array_size = PyArray_SHAPE((PyArrayObject *)p_data)[0];
            for (size_t i = 0; i < array_size; ++i)
            {
                p_f32data[i] *= scale_factor;
            }
            break;
        case FLOAT64:
            scale_factor = (double)scale_factor;
            double *p_f64data = (double *const)PyArray_DATA((PyArrayObject *)p_data);
            array_size = PyArray_SHAPE((PyArrayObject *)p_data)[0];
            for (size_t i = 0; i < array_size; ++i)
            {
                p_f64data[i] *= scale_factor;
            }
            break;
        }
    }

    return p_data;
}

static PyObject *add_data_in_object(PyObject *const p_data, DATA_TYPE data_type, double add_offset)
{
    if (NULL == p_data)
    {
        return NULL;
    }
    else
    {
        switch (data_type)
        {
        case UINT8:
            add_offset = (char)add_offset;
            char *puc_data = (char *const)PyArray_DATA((PyArrayObject *)p_data);
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                puc_data[i] += add_offset;
            }
            break;
        case UINT16:
            add_offset = (unsigned short)add_offset;
            unsigned short *pu16_data = (unsigned short *const)PyArray_DATA((PyArrayObject *)p_data);
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                pu16_data[i] += add_offset;
            }
            break;
        case INT32:
            add_offset = (int)add_offset;
            int *ps32_data = (int *const)PyArray_DATA((PyArrayObject *)p_data);
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                ps32_data[i] += add_offset;
            }
            break;
        case INT64:
            add_offset = (long)add_offset;
            long *ps64_data = (long *const)PyArray_DATA((PyArrayObject *)p_data);
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                ps64_data[i] += add_offset;
            }
            break;
        case FLOAT32:
            add_offset = (float)add_offset;
            float *pf32_data = (float *const)PyArray_DATA((PyArrayObject *)p_data);
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                pf32_data[i] += add_offset;
            }
            break;
        case FLOAT64:
            add_offset = (double)add_offset;
            double *pf64_data = (double *const)PyArray_DATA((PyArrayObject *)p_data);
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                pf64_data[i] += add_offset;
            }
            break;
        }
    }
    return p_data;
}

static PyObject *replace_data_in_object(PyObject *const p_data, DATA_TYPE data_type, double original_data, double new_data)
{
    if (NULL == p_data)
    {
        return NULL;
    }
    else
    {
        switch (data_type)
        {
        case UINT8:
        {
            char *puc_data = ((char *const)PyArray_DATA((PyArrayObject *)p_data));
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                if (puc_data[i] == (char)original_data)
                {
                    puc_data[i] = (char)new_data;
                }
            }
            break;
        }
        case UINT16:
        {
            unsigned short *pu16_data = ((unsigned short *const)PyArray_DATA((PyArrayObject *)p_data));
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                if (pu16_data[i] == (unsigned short)original_data)
                {
                    pu16_data[i] = (unsigned short)new_data;
                }
            }
            break;
        }
        case INT32:
        {
            int *ps32_data = ((int *const)PyArray_DATA((PyArrayObject *)p_data));
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                if (ps32_data[i] == (int)original_data)
                {
                    ps32_data[i] = (int)new_data;
                }
            }
            break;
        }
        case INT64:
        {
            long *p_s64data = ((long *const)PyArray_DATA((PyArrayObject *)p_data));
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                if (p_s64data[i] == (int)original_data)
                {
                    p_s64data[i] = (int)new_data;
                }
            }
            break;
        }
        case FLOAT32:
        {
            float *p_f32ata = ((float *const)PyArray_DATA((PyArrayObject *)p_data));
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                if (p_f32ata[i] == (float)original_data)
                {
                    p_f32ata[i] = (float)new_data;
                }
            }
            break;
        }
        case FLOAT64:
        {
            double *p_f64ata = ((double *const)PyArray_DATA((PyArrayObject *)p_data));
            for (size_t i = 0; i < PyArray_SHAPE((PyArrayObject *)p_data)[0]; ++i)
            {
                if (p_f64ata[i] == (double)original_data)
                {
                    p_f64ata[i] = (double)new_data;
                }
            }
            break;
        }
        }
    }

    return p_data;
}

static double get_timestamp(PyObject *const p_time)
{
    if (p_time != NULL)
    {
        return ((double *const)PyArray_DATA((PyArrayObject *)p_time))[0];
    }
    return -1;
}

static bool parse_ego_motion(st_pickle_ego_motion_type_t *p_st_pickle_ego_motion, PyObject *const p_sp_2021)
{
    bool result = true;
    bool r[25] ;
    if (p_sp_2021 != NULL)
    {
        /* lateral acceleration */
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            // 20211031A: if above signal doesn't exist, try following path for A450_SOP4
	        r[0] = construct_from_f64(&p_st_pickle_ego_motion->lateralAcceleration.lateralAccelerationCentreOfGravity_f64,
				    scale_data(PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                            "egoMotionSignal"),
                                                        "accelerationLateralCOG"),
                                                    FLOAT64, NEGATIVE_CONV),
                                         add_data_in_object(PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                            "egoMotionSignal"),
                                                                "timestamp"),
                                                            FLOAT64, -1 * f64_time_offset_g));


        }

        else
        {
            r[0] = construct_from_f32(&p_st_pickle_ego_motion->lateralAcceleration.lateralAccelerationCentreOfGravity,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "AccelerationLongitudinalLateralProvision"),
                                                            "accelerationLateralCog"),
                                                        "accelerationLateralCog"),
                                                    "accelerationLateralCentreOfGravity"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "AccelerationLongitudinalLateralProvision"),
                                                                "accelerationLateralCog"),
                                                            "timestamp"));
        }
                                                        //FLOAT64, -1 * f64_time_offset_g));

        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[1] = construct_from_f64(&p_st_pickle_ego_motion->lateralAcceleration.lateralAccelerationCentreOfGravityErrorAmplitude_f64,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                        "egoMotionSignal"),
                                                    "accelerationLateralCOGErrorAmp"),
                                                FLOAT64, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                        "egoMotionSignal"),
                                                            "timestamp"));

	    }
	    else
	    {
            r[1] = construct_from_f32(&p_st_pickle_ego_motion->lateralAcceleration.lateralAccelerationCentreOfGravityErrorAmplitude,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "AccelerationLongitudinalLateralProvision"),
                                                            "accelerationLateralCog"),
                                                        "accelerationLateralCogErrAmp"),
                                                    "accelerationLateralCentreOfGravityErrorAmplitude"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "AccelerationLongitudinalLateralProvision"),
                                             "accelerationLateralCog"),
                                         "timestamp"));
	    }
        // 20211031A: if above signal doesn't exist, try following path for A450_SOP4
        /* potential vector */
        r[2] = construct_from_f32(&p_st_pickle_ego_motion->potentialVector.curvatureK,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                 "potVecLateralDynamic"),
                                             "curvatureK"),
                                         "curvatureParameter"),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                                "potVecLateralDynamic"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
        r[3] = construct_from_f32(&p_st_pickle_ego_motion->potentialVector.curvatureA1,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                 "potVecLateralDynamic"),
                                             "curvatureA1"),
                                         "curvatureParameter"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                             "potVecLateralDynamic"),
                                         "timestamp"));
        r[4] = construct_from_f32(&p_st_pickle_ego_motion->potentialVector.curvatureA2,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                 "potVecLateralDynamic"),
                                             "curvatureA2"),
                                         "curvatureParameter"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                             "potVecLateralDynamic"),
                                         "timestamp"));
        r[5] = construct_from_f32(&p_st_pickle_ego_motion->potentialVector.curvatureA3,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                 "potVecLateralDynamic"),
                                             "curvatureA3"),
                                         "curvatureParameter"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                             "potVecLateralDynamic"),
                                         "timestamp"));
        r[6] = construct_from_f32(&p_st_pickle_ego_motion->potentialVector.curvatureB1,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                 "potVecLateralDynamic"),
                                             "curvatureB1"),
                                         "curvatureParameter"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                             "potVecLateralDynamic"),
                                         "timestamp"));
        r[7] = construct_from_f32(&p_st_pickle_ego_motion->potentialVector.curvatureB2,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                 "potVecLateralDynamic"),
                                             "curvatureB2"),
                                         "curvatureParameter"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                             "potVecLateralDynamic"),
                                         "timestamp"));
        r[8] = construct_from_f32(&p_st_pickle_ego_motion->potentialVector.curvatureB3,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                 "potVecLateralDynamic"),
                                             "curvatureB3"),
                                         "curvatureParameter"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                             "potVecLateralDynamic"),
                                         "timestamp"));
        r[9] = construct_from_f32(&p_st_pickle_ego_motion->potentialVector.curvatureB4,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                 "potVecLateralDynamic"),
                                             "curvatureB4"),
                                         "curvatureParameter"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                             "potVecLateralDynamic"),
                                         "timestamp"));
        r[10] = construct_from_f32(&p_st_pickle_ego_motion->potentialVector.curvatureTimeDelay,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                                 "potVecLateralDynamic"),
                                             "curvatureTimeDelay"),
                                         "curvatureTimeDelay"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "PotentialVector2"),
                                             "potVecLateralDynamic"),
                                         "timestamp"));
        /* longitudinal acceleration */
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[11] = construct_from_f64(&p_st_pickle_ego_motion->longitudinalAcceleration.longitudinalAccelerationCentreOfGravity_f64,
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                "egoMotionSignal"),
                                            "accelerationLongitudinalCOG"),
                                        PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                    "egoMotionSignal"),
                                                                "timestamp"));
	    }
	    else
	    {

            r[11] = construct_from_f32(&p_st_pickle_ego_motion->longitudinalAcceleration.longitudinalAccelerationCentreOfGravity,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "AccelerationLongitudinalLateralProvision"),
                                                 "accelerationLongitudinalCog"),
                                             "accelerationLongitudinalCog"),
                                         "accelerationLongitudinalCentreOfGravity"),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "AccelerationLongitudinalLateralProvision"),
                                                                "accelerationLongitudinalCog"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
	    }

        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
        // 20211031A: if above signal doesn't exist, try following path for A450_SOP4
            r[12] = construct_from_f64(&p_st_pickle_ego_motion->longitudinalAcceleration.longitudinalAccelerationCentreOfGravityErrorAmplitude_f64,
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                "egoMotionSignal"),
                                            "accelerationLongitudinalCOGErrorAmp"),
                                        PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                "egoMotionSignal"),
                                                            "timestamp"));
	    }
	    else
	    {

            r[12] = construct_from_f32(&p_st_pickle_ego_motion->longitudinalAcceleration.longitudinalAccelerationCentreOfGravityErrorAmplitude,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "AccelerationLongitudinalLateralProvision"),
                                                 "accelerationLongitudinalCog"),
                                             "accelerationLongitudinalCogErrAmp"),
                                         "accelerationLongitudinalCentreOfGravityErrorAmplitude"),
                                     PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "AccelerationLongitudinalLateralProvision"),
                                                                "accelerationLongitudinalCog"),
                                                            "timestamp"));
	    }
        /* driving direction */
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[13] = construct_from_u8(&p_st_pickle_ego_motion->drivingDirection.drivingDirectionVehicleConfirmed,
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                "egoMotionSignal"),
                                            "drivingDirectionVehicleConfirmed"),
                                        PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                   "egoMotionSignal"),
                                                               "timestamp"));
	    }
	    else
	    {
            r[13] = construct_from_u8(&p_st_pickle_ego_motion->drivingDirection.drivingDirectionVehicleConfirmed,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "VelocityAndVehicleConditionProvision"),
                                            "drivingDirection"),
                                        "drivingDirectionVehicleConfirmed"),
                                    add_data_in_object(PyDict_GetItemWrapper(
                                                           PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(p_sp_2021, "VelocityAndVehicleConditionProvision"),
                                                               "drivingDirection"),
                                                           "timestamp"),
                                                       FLOAT64, -1 * f64_time_offset_g));
	    }


        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
        // 20211031A: if above signal doesn't exist, try following path for A450_SOP4
            r[14] = construct_from_u8(&p_st_pickle_ego_motion->drivingDirection.drivingDirectionVehicleUnconfirmed,
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                "egoMotionSignal"),
                                            "drivingDirectionVehicleUnconfirmed"),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                "egoMotionSignal"),
                                            "timestamp"));
	  
	    }
	    else
	    {
            r[14] = construct_from_u8(&p_st_pickle_ego_motion->drivingDirection.drivingDirectionVehicleUnconfirmed,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "VelocityAndVehicleConditionProvision"),
                                            "drivingDirection"),
                                        "drivingDirectionVehicleUnconfirmed"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "VelocityAndVehicleConditionProvision"),
                                            "drivingDirection"),
                                        "timestamp"));
	    }
        /* vehicle status */
        r[15] = construct_from_u8(&p_st_pickle_ego_motion->vehicleStatus.statusConditionVehicle,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "VehicleCondition"),
                                            "VehicleCondition"),
                                        "statusConditionVehicle"),
                                    add_data_in_object(PyDict_GetItemWrapper(
                                                           PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(p_sp_2021, "VehicleCondition"),
                                                               "VehicleCondition"),
                                                           "timestamp"),
                                                       FLOAT64, -1 * f64_time_offset_g));
        /* vehicle speed */
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[16] = construct_from_f64(&p_st_pickle_ego_motion->vehicleSpeed.speedVehicleCentreOfGravity_f64,
                                         scale_data(PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                "egoMotionSignal"),
                                                            "velocityVehicleCOG"),
                                                    FLOAT64, KMH_TO_MS),
                                        PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                    "egoMotionSignal"),
                                                                "timestamp"));
	    }
	    else
	    {
	        r[16] = construct_from_f32(&p_st_pickle_ego_motion->vehicleSpeed.speedVehicleCentreOfGravity,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "VelocityAndVehicleConditionProvision"),
                                                            "velocityVehicle"),
                                                        "velocityVehicleCog"),
                                                    "velocityVehicleCog"),
                                                FLOAT32, KMH_TO_MS),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "VelocityAndVehicleConditionProvision"),
                                                                "velocityVehicle"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
	    }
        /* yaw speed */
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[17] = construct_from_f64(&p_st_pickle_ego_motion->yawSpeed.yawVelocityVehicle_f64,
                                        scale_data(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                "egoMotionSignal"),
                                                            "yawRate"),
                                                    FLOAT64, NEGATIVE_CONV),
                                        PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                    "egoMotionSignal"),
                                                                "timestamp"));
        
	    }
	    else
	    {
            r[17] = construct_from_f32(&p_st_pickle_ego_motion->yawSpeed.yawVelocityVehicle,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "YawRateProvision"),
                                                            "yawRate"),
                                                        "yawRate"),
                                                    "yawRate"),
                                                FLOAT32, NEGATIVE_CONV),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "YawRateProvision"),
                                                                "yawRate"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
	    }
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[18] = construct_from_f64(&p_st_pickle_ego_motion->yawSpeed.yawVelocityVehicleErrorAmplitude_f64,
                                         scale_data(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                "egoMotionSignal"),
                                                        "yawRateErrAmp"),
                                                    FLOAT64, NEGATIVE_CONV),
                                        PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                    "egoMotionSignal"),
                                                                "timestamp"));

	    }
	    else
	    {
            r[18] = construct_from_f32(&p_st_pickle_ego_motion->yawSpeed.yawVelocityVehicleErrorAmplitude,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "YawRateProvision"),
                                                            "yawRate"),
                                                        "yawRateErrAmp"),
                                                    "yawRateErrAmp"),
                                                FLOAT32, NEGATIVE_CONV),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "YawRateProvision"),
                                                                "yawRate"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));        
	    }
        /* curvature and slip side angle */
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[19] = construct_from_f64(&p_st_pickle_ego_motion->curvatureAndSlipSideAngle.ActualValueCurvatureVehicleMotion_f64,
                                         scale_data(PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                            "egoMotionOdo"),
                                                        "curvatureOdometry"),
                                                    FLOAT64, NEGATIVE_CONV),
                                         add_data_in_object(PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                                    "egoMotionOdo"),
                                                                "timestamp"),
                                                            FLOAT64, -1 * f64_time_offset_g));        
	    }
	    else
	    {
            r[19] = construct_from_f32(&p_st_pickle_ego_motion->curvatureAndSlipSideAngle.ActualValueCurvatureVehicleMotion,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                                            "curvatureAndSlipSideAngle"),
                                                        "curvature"),
                                                    "ActualValueCurvatureVehicleMotion"),
                                                FLOAT32, NEGATIVE_CONV),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                                                "curvatureAndSlipSideAngle"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
	    }
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[20] = construct_from_f64(&p_st_pickle_ego_motion->curvatureAndSlipSideAngle.ActualValueSlipSideAngleVehicleMotion_f64,
                                         scale_data(PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                            "egoMotionOdo"),
                                                        "slipAngleOdometry"),
                                                    FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                 "egoMotionOdo"),
                                             "timestamp"));            
	    }
	    else
	    {
            r[20] = construct_from_f32(&p_st_pickle_ego_motion->curvatureAndSlipSideAngle.ActualValueSlipSideAngleVehicleMotion,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                                            "curvatureAndSlipSideAngle"),
                                                        "slipSideAngle"),
                                                    "ActualValueSlipSideAngleVehicleMotion"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                             "curvatureAndSlipSideAngle"),
                                         "timestamp"));
	    }
	
        /* odo values */
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[21] = construct_from_f64(&p_st_pickle_ego_motion->odoValues.xPositionOdometryVehicle_f64,
                                         add_data_in_object(PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                                    "egoMotionOdo"),
                                                                "xPositionOdometry"),
                                                            FLOAT64, -1. * codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE),
                                        PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                                    "egoMotionOdo"),
                                                                "timestamp"));
	    }
	    else
	    {
	    
            r[21] = construct_from_f32(&p_st_pickle_ego_motion->odoValues.xPositionOdometryVehicle,
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                                                    "odometryValues"),
                                                                "xPosition"),
                                                            "xPositionOdometryVehicle"),
                                                        FLOAT32, -1. * codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                                                "odometryValues"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
	    }
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[22] = construct_from_f64(&p_st_pickle_ego_motion->odoValues.yPositionOdometryVehicle_f64,
                                         scale_data(PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                            "egoMotionOdo"),
                                                        "yPositionOdometry"),
                                                    FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                 "egoMotionOdo"),
                                             "timestamp"));            
	    }
	    else
	    {
            r[22] = construct_from_f32(&p_st_pickle_ego_motion->odoValues.yPositionOdometryVehicle,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                                            "odometryValues"),
                                                        "yPosition"),
                                                    "yPositionOdometryVehicle"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                             "odometryValues"),
                                         "timestamp"));
	    }
	
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[23] = construct_from_f64(&p_st_pickle_ego_motion->odoValues.yawAngleOdometryVehicle_f64,
                                         scale_data(PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                            "egoMotionOdo"),
                                                        "yawAngleOdometry"),
                                                    FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "EgoMotionOdometry"),
                                                 "egoMotionOdo"),
                                             "timestamp"));
	    }
	    else
	    {
            r[23] = construct_from_f32(&p_st_pickle_ego_motion->odoValues.yawAngleOdometryVehicle,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                                            "odometryValues"),
                                                        "yawAngle"),
                                                    "YawAngleOdometryVehicle"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "CentralOdometryProvision"),
                                             "odometryValues"),
                                         "timestamp"));
	    }
	
        /* tacho speed */ /// same as speedVehicleCentreOfGravity, already scaled by KMH_TO_MS
        /// time offset already done in speedVehicleCentreOfGravity
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[24] = construct_from_f64(&p_st_pickle_ego_motion->tachoSpeed.displayedValueSpeed_f64,
                                         PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                "egoMotionSignal"),
                                            "velocityVehicleCOG"),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                 "egoMotionSignal"),
                                             "timestamp"));
	    }
	    else
	    {
            r[24] = construct_from_f32(&p_st_pickle_ego_motion->tachoSpeed.displayedValueSpeed,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "VelocityAndVehicleConditionProvision"),
                                                 "velocityVehicle"),
                                             "velocityVehicleCog"),
                                         "velocityVehicleCog"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "VelocityAndVehicleConditionProvision"),
                                             "velocityVehicle"),
                                         "timestamp"));
	    }
	
        result = true ;
        for( int n = 0 ; n <= 24 ; n++ )
        {
            result &= r[n] ;
        }
    }
    else
    {
        result = false;
    }
    
    return result;
}

static bool parse_vision_lane_support_host_lanes(st_pickle_vision_lane_support_t *const st_vision_lane_support, PyObject *const p_spi_eyeq_core_lanes_host)
{
    bool result = true;
    if (p_spi_eyeq_core_lanes_host != NULL)
    {
        result &= parse_support_lane_host_left(&st_vision_lane_support->host_left, p_spi_eyeq_core_lanes_host);
        result &= parse_support_lane_host_right(&st_vision_lane_support->host_right, p_spi_eyeq_core_lanes_host);
        // // TODO Investigate comment in Matlab: % Reassign correct lane by identifying them via "Side"-attribute
        // reassign_host_lanes_left_right(&st_vision_lane_support->host_left, &st_vision_lane_support->host_right);
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_vision_lane_support_adj_lanes(st_pickle_vision_lane_support_t *const st_vision_lane_support, PyObject *const p_spi_eyeq_core_lanes_adjacent)
{
    bool result = true;
    if (p_spi_eyeq_core_lanes_adjacent != NULL)
    {
        result &= parse_support_lane_adj_left(&st_vision_lane_support->adj_left, p_spi_eyeq_core_lanes_adjacent);
        result &= parse_support_lane_adj_right(&st_vision_lane_support->adj_right, p_spi_eyeq_core_lanes_adjacent);
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_support_lane_host_left(st_pickle_vision_lane_support_lane_t *st_support_lane, PyObject *const p_spi_eyeq_core_lanes_host)
{

    bool result = true;
    if (p_spi_eyeq_core_lanes_host != NULL)
    {
        PyObject *p_timestamp = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "timestamp");
        /// Subtract f64_time_offset_g offset
        p_timestamp = add_data_in_object(p_timestamp, FLOAT64, -1 * f64_time_offset_g);

        PyObject *const p_side = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Side_0");
        /// set vision_lane_support_data_type
        PyArray_Descr *type = PyArray_DESCR((PyArrayObject *)p_side);
        if (strcmp(type->typeobj->tp_name, "numpy.int32") == 0)
        {
            vision_lane_support_data_type = INT32;
        }
        else if (strcmp(type->typeobj->tp_name, "numpy.int64") == 0)
        {
            vision_lane_support_data_type = INT64;
        }

        PyObject *const p_track_id = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Track_ID_0");
        PyObject *const p_age = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Age_0");
        PyObject *const p_availability_state = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Availability_State_0");
        PyObject *const p_prediction_reason = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Prediction_Reason_0");
        PyObject *const p_existence_prob = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Confidence_0");
        PyObject *const p_color = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Color_0");

        //Set to zero
        if (vision_lane_support_data_type == INT32)
        {
            PyObject *const p_prediction_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Prediction_Reason_0");
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Prediction_Type, p_prediction_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            PyObject *const p_prediction_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Prediction_Reason_0");
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Prediction_Type, p_prediction_type, p_timestamp);
        }

        PyObject *const p_classification = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Lanemark_Type_0");
        PyObject *const p_dlm_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_DLM_Type_0");
        PyObject *const p_decel_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_DECEL_Type_0");
        PyObject *const p_view_range_start = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_First_VR_Start_0");
        PyObject *const p_view_range_end = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_First_VR_End_0");
        //Set to boolean in matlab
        PyObject *const p_crossing = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Crossing_0");
        PyObject *const p_marker_width = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Marker_Width_0");
        PyObject *const p_marker_width_std = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Marker_Width_STD_0");
        PyObject *const p_line_c0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C0_0");
        //Set to zero
        // PyObject *const p_line_c0_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C0_0"), FLOAT64, 0.0);
        PyObject *const p_line_c1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C1_0");
        //Set to zero
        // PyObject *const p_line_c1_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C1_0"), FLOAT64, 0.0);
        PyObject *const p_line_c2 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C2_0");
        //Set to zero
        // PyObject *const p_line_c2_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C2_0"), FLOAT64, 0.0);
        PyObject *const p_line_c3 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C3_0");
        //Set to zero
        // PyObject *const p_line_c3_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C3_0"), FLOAT64, 0.0);

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Track_ID, p_track_id, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Track_ID, p_track_id, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Age, p_age, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Age, p_age, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Availability_State, p_availability_state, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Availability_State, p_availability_state, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Prediction_Reason, p_prediction_reason, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Prediction_Reason, p_prediction_reason, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Side, p_side, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Side, p_side, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Color, p_color, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Color, p_color, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Classification, p_classification, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Classification, p_classification, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.DLM_Type, p_dlm_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.DLM_Type, p_dlm_type, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.DECEL_Type, p_decel_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.DECEL_Type, p_decel_type, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Crossing, p_crossing, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Crossing, p_crossing, p_timestamp);
        }

        result &= construct_from_f64(&st_support_lane->Existence_Probability, p_existence_prob, p_timestamp);
        result &= construct_from_f64(&st_support_lane->View_Range_Start, p_view_range_start, p_timestamp);
        result &= construct_from_f64(&st_support_lane->View_Range_End, p_view_range_end, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Marker_Width, p_marker_width, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Marker_Width_STD, p_marker_width_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C0, p_line_c0, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C0_STD, p_line_c0_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C1, p_line_c1, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C1_STD, p_line_c1_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C2, p_line_c2, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C2_STD, p_line_c2_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C3, p_line_c3, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C3_STD, p_line_c3_std, p_timestamp);

        // set global null struct set_global_f64_pickle_struct for the first and only time
        // will use Line_C0 length and time
        set_global_f64_pickle_struct(st_support_lane->Line_C0.u32_length, st_support_lane->Line_C0.f64_time_min, st_support_lane->Line_C0.f64_time_max, st_support_lane->Line_C0.pf64_timestamp);
        assign_f64_pickle_struct((&st_support_lane->Line_C0_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct((&st_support_lane->Line_C1_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct((&st_support_lane->Line_C2_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct((&st_support_lane->Line_C3_STD), &st_lca_pickle_base_f64_g);
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_support_lane_host_right(st_pickle_vision_lane_support_lane_t *st_support_lane, PyObject *const p_spi_eyeq_core_lanes_host)
{
    bool result = true;
    if (p_spi_eyeq_core_lanes_host != NULL)
    {
        PyObject *p_timestamp = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "timestamp");
        /// Subtract f64_time_offset_g offset -- already done
        // p_timestamp = add_data_in_object(p_timestamp, FLOAT64, -1 * f64_time_offset_g);

        PyObject *const p_side = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Side_1");
        PyObject *const p_track_id = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Track_ID_1");
        PyObject *const p_age = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Age_1");
        PyObject *const p_availability_state = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Availability_State_1");
        PyObject *const p_prediction_reason = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Prediction_Reason_1");
        PyObject *const p_existence_prob = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Confidence_1");
        PyObject *const p_color = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Color_1");

        //Set to zero
        if (vision_lane_support_data_type == INT32)
        {
            PyObject *const p_prediction_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Prediction_Reason_1");
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Prediction_Type, p_prediction_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            PyObject *const p_prediction_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Prediction_Reason_1");
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Prediction_Type, p_prediction_type, p_timestamp);
        }

        PyObject *const p_classification = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Lanemark_Type_1");
        PyObject *const p_dlm_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_DLM_Type_1");
        PyObject *const p_decel_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_DECEL_Type_1");
        PyObject *const p_view_range_start = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_First_VR_Start_1");
        PyObject *const p_view_range_end = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_First_VR_End_1");
        PyObject *const p_crossing = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Crossing_1");
        PyObject *const p_marker_width = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Marker_Width_1");
        PyObject *const p_marker_width_std = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Marker_Width_STD_1");
        PyObject *const p_line_c0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C0_1");
        // PyObject *const p_line_c0_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C0_1"), FLOAT64, 0.0);
        //PyObject *const p_line_c0_std //Set to zero
        PyObject *const p_line_c1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C1_1");
        // PyObject *const p_line_c1_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C1_1"), FLOAT64, 0.0);
        //PyObject *const p_line_c1_std //Set to zero
        PyObject *const p_line_c2 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C2_1");
        // PyObject *const p_line_c2_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C2_1"), FLOAT64, 0.0);
        //PyObject *const p_line_c2_std  //Set to zero
        PyObject *const p_line_c3 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C3_1");
        // PyObject *const p_line_c3_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_host, "LH_Line_First_C3_1"), FLOAT64, 0.0);
        //PyObject *const p_line_c3_std  //Set to zero

        // Populate struct
        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Track_ID, p_track_id, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Track_ID, p_track_id, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Age, p_age, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Age, p_age, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Availability_State, p_availability_state, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Availability_State, p_availability_state, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Prediction_Reason, p_prediction_reason, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Prediction_Reason, p_prediction_reason, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Side, p_side, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Side, p_side, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Color, p_color, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Color, p_color, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Classification, p_classification, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Classification, p_classification, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.DLM_Type, p_dlm_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.DLM_Type, p_dlm_type, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.DECEL_Type, p_decel_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.DECEL_Type, p_decel_type, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Crossing, p_crossing, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Crossing, p_crossing, p_timestamp);
        }

        result &= construct_from_f64(&st_support_lane->Existence_Probability, p_existence_prob, p_timestamp);
        result &= construct_from_f64(&st_support_lane->View_Range_Start, p_view_range_start, p_timestamp);
        result &= construct_from_f64(&st_support_lane->View_Range_End, p_view_range_end, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Marker_Width, p_marker_width, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Marker_Width_STD, p_marker_width_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C0, p_line_c0, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C0_STD, p_line_c0_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C1, p_line_c1, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C1_STD, p_line_c1_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C2, p_line_c2, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C2_STD, p_line_c2_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C3, p_line_c3, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C3_STD, p_line_c3_std, p_timestamp);

        // set global null struct set_global_f64_pickle_struct for the first and only time
        // will use Line_C0 length and time
        set_global_f64_pickle_struct(st_support_lane->Line_C0.u32_length, st_support_lane->Line_C0.f64_time_min, st_support_lane->Line_C0.f64_time_max, st_support_lane->Line_C0.pf64_timestamp);
        assign_f64_pickle_struct((&st_support_lane->Line_C0_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct((&st_support_lane->Line_C1_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct((&st_support_lane->Line_C2_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct((&st_support_lane->Line_C3_STD), &st_lca_pickle_base_f64_g);
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_support_lane_adj_left(st_pickle_vision_lane_support_adjacent_lane_t *st_support_lane, PyObject *const p_spi_eyeq_core_lanes_adjacent)
{
    bool result = true;
    if (p_spi_eyeq_core_lanes_adjacent != NULL)
    {
        PyObject *p_timestamp = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "timestamp");
        /// Subtract f64_time_offset_g offset
        p_timestamp = add_data_in_object(p_timestamp, FLOAT64, -1 * f64_time_offset_g);

        PyObject *const p_side = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_Side_0");

        PyObject *const p_track_id = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Lane_Track_ID_0");
        PyObject *const p_age = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Age_0");
        PyObject *const p_availability_state = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Availability_State_0");
        PyObject *const p_prediction_reason = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Prediction_Reason_0");
        PyObject *const p_existence_prob = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Confidence_0");
        PyObject *const p_color = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Color_0");
        //Set to zero
        if (vision_lane_support_data_type == INT32)
        {
            PyObject *const p_prediction_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Prediction_Reason_0");
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Prediction_Type, p_prediction_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            PyObject *const p_prediction_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Prediction_Reason_0");
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Prediction_Type, p_prediction_type, p_timestamp);
        }

        PyObject *const p_classification = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Lanemark_Type_0");
        PyObject *const p_dlm_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_DLM_Type_0");
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_decel_type = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_DLM_Type_0"), INT64, 0.0);
        PyObject *const p_view_range_start = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_View_Range_Start_0");
        PyObject *const p_view_range_end = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_View_Range_End_0");
        //Set to zero through the common zero pickle_struct
        //Set to Zero TODO Investigate comment in matlab script
        // PyObject *const p_crossing = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_DLM_Type_0"), INT64, 0.0);
        PyObject *const p_marker_width = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Marker_Width_0");
        PyObject *const p_marker_width_std = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Marker_Width_STD_0");
        PyObject *const p_line_c0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C0_0");
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_line_c0_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C0_0"), FLOAT64, 0.0);
        PyObject *const p_line_c1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C1_0");
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_line_c1_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C1_0"), FLOAT64, 0.0);
        PyObject *const p_line_c2 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C2_0");
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_line_c2_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C2_0"), FLOAT64, 0.0);
        PyObject *const p_line_c3 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C3_0");
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_line_c3_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C3_0"), FLOAT64, 0.0);

        // Populate struct
        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Track_ID, p_track_id, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Track_ID, p_track_id, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Age, p_age, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Age, p_age, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Availability_State, p_availability_state, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Availability_State, p_availability_state, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Prediction_Reason, p_prediction_reason, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Prediction_Reason, p_prediction_reason, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Side, p_side, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Side, p_side, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Color, p_color, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Color, p_color, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Classification, p_classification, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Classification, p_classification, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.DLM_Type, p_dlm_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.DLM_Type, p_dlm_type, p_timestamp);
        }

        result &= construct_from_f64(&st_support_lane->Existence_Probability, p_existence_prob, p_timestamp);
        // result &= construct_from_s64(&st_support_lane->DECEL_Type, p_decel_type, p_timestamp);
        result &= construct_from_f64(&st_support_lane->View_Range_Start, p_view_range_start, p_timestamp);
        result &= construct_from_f64(&st_support_lane->View_Range_End, p_view_range_end, p_timestamp);
        // result &= construct_from_s64(&st_support_lane->Crossing, p_crossing, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Marker_Width, p_marker_width, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Marker_Width_STD, p_marker_width_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C0, p_line_c0, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C0_STD, p_line_c0_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C1, p_line_c1, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C1_STD, p_line_c1_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C2, p_line_c2, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C2_STD, p_line_c2_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C3, p_line_c3, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C3_STD, p_line_c3_std, p_timestamp);

        // set global null struct set_global_s64_pickle_struct for the first and only time
        // will use DLM type length and time
        if (vision_lane_support_data_type == INT32)
        {
            set_global_s32_pickle_struct(st_support_lane->vision_lane_set_s32.DLM_Type.u32_length, st_support_lane->vision_lane_set_s32.DLM_Type.f64_time_min, st_support_lane->vision_lane_set_s32.DLM_Type.f64_time_max, st_support_lane->vision_lane_set_s32.DECEL_Type.pf64_timestamp);
            assign_s32_pickle_struct(&(st_support_lane->vision_lane_set_s32.DECEL_Type), &st_lca_pickle_base_s32_g);
            assign_s32_pickle_struct(&(st_support_lane->vision_lane_set_s32.Crossing), &st_lca_pickle_base_s32_g);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            set_global_s64_pickle_struct(st_support_lane->vision_lane_set_s64.DLM_Type.u32_length, st_support_lane->vision_lane_set_s64.DLM_Type.f64_time_min, st_support_lane->vision_lane_set_s64.DLM_Type.f64_time_max, st_support_lane->vision_lane_set_s64.DECEL_Type.pf64_timestamp);
            assign_s64_pickle_struct(&(st_support_lane->vision_lane_set_s64.DECEL_Type), &st_lca_pickle_base_s64_g);
            assign_s64_pickle_struct(&(st_support_lane->vision_lane_set_s64.Crossing), &st_lca_pickle_base_s64_g);
        }

        assign_f64_pickle_struct(&(st_support_lane->Line_C0_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(st_support_lane->Line_C1_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(st_support_lane->Line_C2_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(st_support_lane->Line_C3_STD), &st_lca_pickle_base_f64_g);
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_support_lane_adj_right(st_pickle_vision_lane_support_adjacent_lane_t *st_support_lane, PyObject *const p_spi_eyeq_core_lanes_adjacent)
{
    bool result = true;
    if (p_spi_eyeq_core_lanes_adjacent != NULL)
    {
        //TODO investigate reshaping
        PyObject *p_timestamp = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "timestamp");
        /// Subtract f64_time_offset_g offset -- already done
        // p_timestamp = add_data_in_object(p_timestamp, FLOAT64, -1 * f64_time_offset_g);

        PyObject *const p_side = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_Side_1");
        PyObject *const p_track_id = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Lane_Track_ID_1");
        PyObject *const p_age = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Age_1");
        PyObject *const p_availability_state = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Availability_State_1");
        PyObject *const p_prediction_reason = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Prediction_Reason_1");
        PyObject *const p_existence_prob = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Confidence_1");
        PyObject *const p_color = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Color_1");

        //Set to zero
        if (vision_lane_support_data_type == INT32)
        {
            PyObject *const p_prediction_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Prediction_Reason_1");
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Prediction_Type, p_prediction_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            PyObject *const p_prediction_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Prediction_Reason_1");
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Prediction_Type, p_prediction_type, p_timestamp);
        }

        PyObject *const p_classification = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_DLM_Type_1");
        PyObject *const p_dlm_type = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_DLM_Type_1");
        // PyObject *const p_decel_type; //Set to zero
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_decel_type = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_DLM_Type_1"), INT64, 0.0);
        PyObject *const p_view_range_start = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_View_Range_Start_1");
        PyObject *const p_view_range_end = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_View_Range_End_1");
        // PyObject *const p_crossing; //Set to Zero TODO Investigate comment in matlab script
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_crossing = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_DLM_Type_1"), INT64, 0.0);
        PyObject *const p_marker_width = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Marker_Width_1");
        PyObject *const p_marker_width_std = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Marker_Width_STD_1");
        PyObject *const p_line_c0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C0_1");
        // PyObject *const p_line_c0_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C0_1"), FLOAT64, 0.0);

        //PyObject *const p_line_c0_std //Set to zero
        PyObject *const p_line_c1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C1_1");
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_line_c1_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C1_1"), FLOAT64, 0.0);

        //PyObject *const p_line_c1_std //Set to zero
        PyObject *const p_line_c2 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C2_1");
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_line_c2_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C2_1"), FLOAT64, 0.0);

        //PyObject *const p_line_c2_std  //Set to zero
        PyObject *const p_line_c3 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C3_1");
        //Set to zero through the common zero pickle_struct
        // PyObject *const p_line_c3_std = scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "LA_Line_C3_1"), FLOAT64, 0.0);

        // Populate struct
        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Track_ID, p_track_id, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Track_ID, p_track_id, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Age, p_age, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Age, p_age, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Availability_State, p_availability_state, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Availability_State, p_availability_state, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Prediction_Reason, p_prediction_reason, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Prediction_Reason, p_prediction_reason, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Side, p_side, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Side, p_side, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Color, p_color, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Color, p_color, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.Classification, p_classification, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.Classification, p_classification, p_timestamp);
        }

        if (vision_lane_support_data_type == INT32)
        {
            result &= construct_from_s32(&st_support_lane->vision_lane_set_s32.DLM_Type, p_dlm_type, p_timestamp);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            result &= construct_from_s64(&st_support_lane->vision_lane_set_s64.DLM_Type, p_dlm_type, p_timestamp);
        }

        result &= construct_from_f64(&st_support_lane->Existence_Probability, p_existence_prob, p_timestamp);
        // result &= construct_from_s64(&st_support_lane->DECEL_Type, p_decel_type, p_timestamp);
        result &= construct_from_f64(&st_support_lane->View_Range_Start, p_view_range_start, p_timestamp);
        result &= construct_from_f64(&st_support_lane->View_Range_End, p_view_range_end, p_timestamp);
        // result &= construct_from_s64(&st_support_lane->Crossing, p_crossing, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Marker_Width, p_marker_width, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Marker_Width_STD, p_marker_width_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C0, p_line_c0, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C0_STD, p_line_c0_std,p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C1, p_line_c1, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C1_STD, p_line_c1_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C2, p_line_c2, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C2_STD, p_line_c2_std, p_timestamp);
        result &= construct_from_f64(&st_support_lane->Line_C3, p_line_c3, p_timestamp);
        // result &= construct_from_f64(&st_support_lane->Line_C3_STD, p_line_c3_std, p_timestamp);

        if (vision_lane_support_data_type == INT32)
        {
            assign_s32_pickle_struct(&(st_support_lane->vision_lane_set_s32.DECEL_Type), &st_lca_pickle_base_s32_g);
            assign_s32_pickle_struct(&(st_support_lane->vision_lane_set_s32.Crossing), &st_lca_pickle_base_s32_g);
        }
        else if (vision_lane_support_data_type == INT64)
        {
            assign_s64_pickle_struct(&(st_support_lane->vision_lane_set_s64.DECEL_Type), &st_lca_pickle_base_s64_g);
            assign_s64_pickle_struct(&(st_support_lane->vision_lane_set_s64.Crossing), &st_lca_pickle_base_s64_g);
        }

        assign_f64_pickle_struct(&(st_support_lane->Line_C0_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(st_support_lane->Line_C1_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(st_support_lane->Line_C2_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(st_support_lane->Line_C3_STD), &st_lca_pickle_base_f64_g);
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_vision_lane_support_adv(st_pickle_vision_lane_support_adv_t *p_st_lane_support_adv, PyObject *const p_spi_eyeq_core_lanes_application)
{
    bool result = true;
    if (p_spi_eyeq_core_lanes_application != NULL)
    {
        /// set vision_lane_support_adv_data_type a 
        PyArray_Descr *type_a = PyArray_DESCR((PyArrayObject *)PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Construction_Area"));
        if (strcmp(type_a->typeobj->tp_name, "numpy.int32") == 0)
        {
            vision_lane_support_adv_data_type_a = INT32;
        }
        else if (strcmp(type_a->typeobj->tp_name, "numpy.int64") == 0)
        {
            vision_lane_support_adv_data_type_a = INT64;
        }

        /// set vision_lane_support_adv_data_type b
        PyArray_Descr *type_b = PyArray_DESCR((PyArrayObject *)PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Available"));
        if (strcmp(type_b->typeobj->tp_name, "numpy.float32") == 0)
        {
            vision_lane_support_adv_data_type_b = FLOAT32;
        }
        else if (strcmp(type_b->typeobj->tp_name, "numpy.float64") == 0)
        {
            vision_lane_support_adv_data_type_b = FLOAT64;
        }


        if (vision_lane_support_adv_data_type_a == INT32)
        {
            result &= construct_from_s32(&p_st_lane_support_adv->vision_lane_adv_set32_a.LS_CA_Is_Construction_Area,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Construction_Area"),
                                         add_data_in_object(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
            result &= construct_from_s32(&p_st_lane_support_adv->vision_lane_adv_set32_a.LS_INTP_Is_Highway_Merge_Left,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Highway_Merge_Left"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_adv->vision_lane_adv_set32_a.LS_INTP_Is_Highway_Merge_Right,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Highway_Merge_Right"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_adv->vision_lane_adv_set32_a.LS_INTP_Is_Highway_Exit_Left,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Highway_Exit_Left"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_adv->vision_lane_adv_set32_a.LS_INTP_Is_Highway_Exit_Right,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Highway_Exit_Right"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_adv->vision_lane_adv_set32_a.LS_INTP_Is_Valid_0,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_INTP_Is_Valid_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_adv->vision_lane_adv_set32_a.LS_INTP_Snow_On_Road,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_INTP_SRD_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_adv->vision_lane_adv_set32_a.LAP_Path_Pred_First_Valid,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_Valid"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_adv->vision_lane_adv_set32_a.LAP_Path_Pred_Second_Valid,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Second_Valid"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
        }
        else if (vision_lane_support_adv_data_type_a == INT64)
        {
            result &= construct_from_s64(&p_st_lane_support_adv->vision_lane_adv_set64_a.LS_CA_Is_Construction_Area,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Construction_Area"),
                                         add_data_in_object(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
            result &= construct_from_s64(&p_st_lane_support_adv->vision_lane_adv_set64_a.LS_INTP_Is_Highway_Merge_Left,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Highway_Merge_Left"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_adv->vision_lane_adv_set64_a.LS_INTP_Is_Highway_Merge_Right,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Highway_Merge_Right"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_adv->vision_lane_adv_set64_a.LS_INTP_Is_Highway_Exit_Left,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Highway_Exit_Left"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_adv->vision_lane_adv_set64_a.LS_INTP_Is_Highway_Exit_Right,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Is_Highway_Exit_Right"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_adv->vision_lane_adv_set64_a.LS_INTP_Is_Valid_0,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_INTP_Is_Valid_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_adv->vision_lane_adv_set64_a.LS_INTP_Snow_On_Road,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_INTP_SRD_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_adv->vision_lane_adv_set64_a.LAP_Path_Pred_First_Valid,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_Valid"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_adv->vision_lane_adv_set64_a.LAP_Path_Pred_Second_Valid,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Second_Valid"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
        }


        if (vision_lane_support_adv_data_type_b == FLOAT32)
        {
            result &= construct_from_f32(&p_st_lane_support_adv->vision_lane_adv_set32_b.LAP_Path_Pred_Available,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Available"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f32(&p_st_lane_support_adv->vision_lane_adv_set32_b.LAP_Path_Pred_Conf,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Conf"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f32(&p_st_lane_support_adv->vision_lane_adv_set32_b.LAP_Path_Pred_First_C0,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_C0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f32(&p_st_lane_support_adv->vision_lane_adv_set32_b.LAP_Path_Pred_First_C1,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_C1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f32(&p_st_lane_support_adv->vision_lane_adv_set32_b.LAP_Path_Pred_First_C2,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_C2"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f32(&p_st_lane_support_adv->vision_lane_adv_set32_b.LAP_Path_Pred_First_C3,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_C3"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f32(&p_st_lane_support_adv->vision_lane_adv_set32_b.LAP_Path_Pred_First_VR_End,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_VR_End"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f32(&p_st_lane_support_adv->vision_lane_adv_set32_b.LAP_Path_Pred_Second_VR_End,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_second_VR_End"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f32(&p_st_lane_support_adv->vision_lane_adv_set32_b.LAP_Path_Pred_Half_Width,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Half_Width"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
        }
        else if (vision_lane_support_adv_data_type_b == FLOAT64)
        {
            result &= construct_from_f64(&p_st_lane_support_adv->vision_lane_adv_set64_b.LAP_Path_Pred_Available,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Available"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f64(&p_st_lane_support_adv->vision_lane_adv_set64_b.LAP_Path_Pred_Conf,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Conf"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f64(&p_st_lane_support_adv->vision_lane_adv_set64_b.LAP_Path_Pred_First_C0,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_C0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f64(&p_st_lane_support_adv->vision_lane_adv_set64_b.LAP_Path_Pred_First_C1,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_C1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f64(&p_st_lane_support_adv->vision_lane_adv_set64_b.LAP_Path_Pred_First_C2,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_C2"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f64(&p_st_lane_support_adv->vision_lane_adv_set64_b.LAP_Path_Pred_First_C3,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_C3"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f64(&p_st_lane_support_adv->vision_lane_adv_set64_b.LAP_Path_Pred_First_VR_End,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_First_VR_End"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f64(&p_st_lane_support_adv->vision_lane_adv_set64_b.LAP_Path_Pred_Second_VR_End,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Second_VR_End"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
            result &= construct_from_f64(&p_st_lane_support_adv->vision_lane_adv_set64_b.LAP_Path_Pred_Half_Width,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "LAP_Path_Pred_Half_Width"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_application, "timestamp"));
        }

    }
    else
    {
        result = false;
    }
    return result;
}

///@TODO: The values are set to zeros in import_pickle_data_versions.m, will be parsed but not used
///       when updating the model inputs
static bool parse_vision_lane_support_semantic_lane(st_pickle_vision_lane_support_semantic_lane_t *p_st_lane_support_semantic_lane, PyObject *const p_spi_eyeq_core_semantic_lanes)
{
    bool result = true;
    if (p_spi_eyeq_core_semantic_lanes != NULL)
    {
        result &= construct_from_f64(&p_st_lane_support_semantic_lane->host.SLD_existence_probability,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Existence_Probability_0"),
                                     add_data_in_object(PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"), FLOAT64, -1 * f64_time_offset_g));

        /// set vision_lane_support_adv_data_type
        PyArray_Descr *type = PyArray_DESCR((PyArrayObject *)PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_0"));
        if (strcmp(type->typeobj->tp_name, "numpy.int32") == 0)
        {
            vision_lane_semantic_lane_data_type = INT32;
        }
        else if (strcmp(type->typeobj->tp_name, "numpy.int64") == 0)
        {
            vision_lane_semantic_lane_data_type = INT64;
        }

        if (vision_lane_semantic_lane_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_lane_support_semantic_lane->host.vision_set32.SLD_orientation,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_semantic_lane->host.vision_set32.SLD_type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Type_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_semantic_lane->adj_right.vision_set32.SLD_orientation,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_semantic_lane->adj_right.vision_set32.SLD_type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Type_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_semantic_lane->adj_left.vision_set32.SLD_orientation,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s32(&p_st_lane_support_semantic_lane->adj_left.vision_set32.SLD_type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Type_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
        }
        else if (vision_lane_semantic_lane_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_lane_support_semantic_lane->host.vision_set64.SLD_orientation,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_semantic_lane->host.vision_set64.SLD_type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Type_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_semantic_lane->adj_right.vision_set64.SLD_orientation,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_semantic_lane->adj_right.vision_set64.SLD_type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Type_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_semantic_lane->adj_left.vision_set64.SLD_orientation,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
            result &= construct_from_s64(&p_st_lane_support_semantic_lane->adj_left.vision_set64.SLD_type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Type_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
        }
        result &= construct_from_f64(&p_st_lane_support_semantic_lane->host.SLD_orientation_probability,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_Probability_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
        result &= construct_from_f64(&p_st_lane_support_semantic_lane->adj_right.SLD_existence_probability,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Existence_Probability_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
        result &= construct_from_f64(&p_st_lane_support_semantic_lane->adj_right.SLD_orientation_probability,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_Probability_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
        result &= construct_from_f64(&p_st_lane_support_semantic_lane->adj_left.SLD_existence_probability,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Existence_Probability_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
        result &= construct_from_f64(&p_st_lane_support_semantic_lane->adj_left.SLD_orientation_probability,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "SLD_Orientation_Probability_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_semantic_lanes, "timestamp"));
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_partner_function_output(st_pickle_partner_function_output_t *p_st_partner_function_output, PyObject *const p_spi_2021)
{
    bool result = true;
    if (p_spi_2021 != NULL)
    {
        PyObject *lateral_functions_status = PyDict_GetItemWrapper(PyDict_GetItemWrapper(p_spi_2021, "LateralFunctionsStatus"), "lateralFunctionsStatus");
        result &= construct_from_u8(&p_st_partner_function_output->LSAStatus, PyDict_GetItemWrapper(PyDict_GetItemWrapper(lateral_functions_status, "qualifierFunctionLSA"), "qualifierFunctionLSA"),
                                    add_data_in_object(PyDict_GetItemWrapper(lateral_functions_status, "timestamp"), FLOAT64, -1 * f64_time_offset_g));

        ///@TODO: Assign to correct signals as in pickle file once import_pickle_data_versions.m is changed
        // PyObject* lateral_functions_status_and_foresight = PyDict_GetItemWrapper(PyDict_GetItemWrapper(p_spi_2021, "LateralFunctionsStatusAndForesight"), "lateralFunctionsStatusAndForesight");
        // result &= construct_from_u8(&p_st_partner_function_output->SWAStatus, PyDict_GetItemWrapper(PyDict_GetItemWrapper(lateral_functions_status, "laterFunctionsForesightDistance"), "laterFunctionsForesightDistance"),
        //                         PyDict_GetItemWrapper(lateral_functions_status_and_foresight, "timestamp"));
        // result &= construct_from_u8(&p_st_partner_function_output->NoAdmissionCancellation, PyDict_GetItemWrapper(PyDict_GetItemWrapper(lateral_functions_status, "laterFunctionsForesightDistance"), "laterFunctionsForesightDistance"),
        //                         PyDict_GetItemWrapper(lateral_functions_status_and_foresight, "timestamp"));

        /// these 2 values are assigned to zeros for now in import_pickle_data_versions.m
        /// data will be assigned to any value for now
        result &= construct_from_u8(&p_st_partner_function_output->SWAStatus, PyDict_GetItemWrapper(PyDict_GetItemWrapper(lateral_functions_status, "qualifierFunctionLSA"), "qualifierFunctionLSA"),
                                    PyDict_GetItemWrapper(lateral_functions_status, "timestamp"));
        result &= construct_from_u8(&p_st_partner_function_output->NoAdmissionCancellation, PyDict_GetItemWrapper(PyDict_GetItemWrapper(lateral_functions_status, "qualifierFunctionLSA"), "qualifierFunctionLSA"),
                                    PyDict_GetItemWrapper(lateral_functions_status, "timestamp"));
    }
    else
    {
        result = false;
    }
    return result;
}

// static bool parse_vision_lane_CCM(st_pickle_vision_lane_support_CCM_t *p_st_vision_ccm, PyObject *const p_core_lanes_ccm_protocol)
// {
//     bool result = true;
//     if (p_core_lanes_ccm_protocol != NULL)
//     {
//         PyObject *p_timestamp = PyDict_GetItemWrapper(p_core_lanes_ccm_protocol, "timestamp");
//         /// Subtract f64_time_offset_g offset
//         p_timestamp = add_data_in_object(p_timestamp, FLOAT64, -1 * f64_time_offset_g);

//         PyObject *const p_ccm_obj_count = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Objects_Count");

//         PyObject *const p_ccm_obj_lm_id_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_LM_ID_0");
//         PyObject *const p_ccm_long_pos_1_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_1_0");
//         PyObject *const p_ccm_estimated_error_1_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_1_0");
//         PyObject *const p_ccm_long_pos_2_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_2_0");
//         PyObject *const p_ccm_estimated_error_2_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_2_0");
//         PyObject *const p_ccm_long_pos_3_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_3_0");
//         PyObject *const p_ccm_estimated_error_3_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_3_0");
//         PyObject *const p_ccm_long_pos_4_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_4_0");
//         PyObject *const p_ccm_estimated_error_4_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_4_0");
//         PyObject *const p_ccm_long_pos_5_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_5_0");
//         PyObject *const p_ccm_estimated_error_5_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_5_0");
//         PyObject *const p_ccm_long_pos_6_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_6_0");
//         PyObject *const p_ccm_estimated_error_6_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_6_0");
//         PyObject *const p_ccm_long_pos_7_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_7_0");
//         PyObject *const p_ccm_estimated_error_7_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_7_0");
//         PyObject *const p_ccm_long_pos_8_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_8_0");
//         PyObject *const p_ccm_estimated_error_8_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_8_0");
//         PyObject *const p_ccm_long_pos_9_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_9_0");
//         PyObject *const p_ccm_estimated_error_9_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_9_0");
//         PyObject *const p_ccm_long_pos_10_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_10_0");
//         PyObject *const p_ccm_estimated_error_10_0 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_10_0");

//         PyObject *const p_ccm_obj_lm_id_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_LM_ID_1");
//         PyObject *const p_ccm_long_pos_1_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_1_1");
//         PyObject *const p_ccm_estimated_error_1_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_1_1");
//         PyObject *const p_ccm_long_pos_2_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_2_1");
//         PyObject *const p_ccm_estimated_error_2_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_2_1");
//         PyObject *const p_ccm_long_pos_3_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_3_1");
//         PyObject *const p_ccm_estimated_error_3_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_3_1");
//         PyObject *const p_ccm_long_pos_4_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_4_1");
//         PyObject *const p_ccm_estimated_error_4_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_4_1");
//         PyObject *const p_ccm_long_pos_5_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_5_1");
//         PyObject *const p_ccm_estimated_error_5_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_5_1");
//         PyObject *const p_ccm_long_pos_6_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_6_1");
//         PyObject *const p_ccm_estimated_error_6_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_6_1");
//         PyObject *const p_ccm_long_pos_7_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_7_1");
//         PyObject *const p_ccm_estimated_error_7_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_7_1");
//         PyObject *const p_ccm_long_pos_8_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_8_1");
//         PyObject *const p_ccm_estimated_error_8_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_8_1");
//         PyObject *const p_ccm_long_pos_9_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_9_1");
//         PyObject *const p_ccm_estimated_error_9_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_9_1");
//         PyObject *const p_ccm_long_pos_10_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Long_Pos_10_1");
//         PyObject *const p_ccm_estimated_error_10_1 = PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_adjacent, "CCM_Estimated_Error_10_1");

//         // Populate struct
//         result &= construct_from_s32(&p_st_vision_ccm->CCM_Objects_Count, p_ccm_obj_count, p_timestamp);

//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_LM_ID, p_ccm_obj_lm_id_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_1_0, p_ccm_long_pos_1_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_1_0, p_ccm_estimated_error_1_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_2_0, p_ccm_long_pos_2_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_2_0, p_ccm_estimated_error_2_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_3_0, p_ccm_long_pos_3_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_3_0, p_ccm_estimated_error_3_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_4_0, p_ccm_long_pos_4_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_4_0, p_ccm_estimated_error_4_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_5_0, p_ccm_long_pos_5_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_5_0, p_ccm_estimated_error_5_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_6_0, p_ccm_long_pos_6_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_6_0, p_ccm_estimated_error_6_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_7_0, p_ccm_long_pos_7_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_7_0, p_ccm_estimated_error_7_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_8_0, p_ccm_long_pos_8_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_8_0, p_ccm_estimated_error_8_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_9_0, p_ccm_long_pos_9_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_9_0, p_ccm_estimated_error_9_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Long_Pos_10_0, p_ccm_long_pos_10_0, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_1.CCM_Estimated_Error_10_0, p_ccm_estimated_error_10_0, p_timestamp);

//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_LM_ID, p_ccm_obj_lm_id_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_1_1, p_ccm_long_pos_1_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_1_1, p_ccm_estimated_error_1_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_2_1, p_ccm_long_pos_2_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_2_1, p_ccm_estimated_error_2_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_3_1, p_ccm_long_pos_3_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_3_1, p_ccm_estimated_error_3_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_4_1, p_ccm_long_pos_4_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_4_1, p_ccm_estimated_error_4_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_5_1, p_ccm_long_pos_5_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_5_1, p_ccm_estimated_error_5_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_6_1, p_ccm_long_pos_6_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_6_1, p_ccm_estimated_error_6_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_7_1, p_ccm_long_pos_7_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_7_1, p_ccm_estimated_error_7_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_8_1, p_ccm_long_pos_8_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_8_1, p_ccm_estimated_error_8_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_9_1, p_ccm_long_pos_9_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_9_1, p_ccm_estimated_error_9_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Long_Pos_10_1, p_ccm_long_pos_10_1, p_timestamp);
//         result &= construct_from_s32(&p_st_vision_ccm->LM_2.CCM_Estimated_Error_10_1, p_ccm_estimated_error_10_1, p_timestamp);
//     }
//     else
//     {
//         result = false;
//     }
//     return result;
// }

static bool parse_vision_object_detection(st_pickle_vision_object_detection_t *p_st_vision_object_detection, PyObject *const p_core_objects_protocol)
{
    bool result = true;
    if (p_core_objects_protocol != NULL)
    {
        bool add_time_offset = true;
        char OBJ_ID_[20];
        char OBJ_Existence_Probability_[40];
        char OBJ_Object_Age_[30];
        char OBJ_Object_Class_[30];
        char OBJ_Class_Probability_[40];
        char OBJ_Car_Probability_[30];
        char OBJ_Truck_Probability_[30];
        char OBJ_Bike_Probability_[30];
        char OBJ_Motion_Status_[30];
        char OBJ_Has_Cut_Lane_[30];
        char OBJ_Has_Cut_Path_[30];
        char OBJ_Lane_Assignment_[40];
        char OBJ_Long_Distance_[30];
        char OBJ_Lat_Distance_[30];
        char OBJ_Relative_Long_Velocity_[40];
        char OBJ_Relative_Long_Velocity_V_[40];
        char OBJ_Relative_Lat_Velocity_[40];
        char OBJ_Relative_Lat_Velocity_V_[40];
        char OBJ_Width_[20];
        char OBJ_Length_[20];
        char OBJ_Heading_[20];
        char OBJ_Inv_TTC_[20];
        char OBJ_Inv_TTC_V_[20];

        for (int i = 0; i < VISION_OBJECT_DETECTION_NUMBER_OF_ELEMENTS; i++)
        {
            sprintf(OBJ_ID_, "OBJ_ID_%d", i);
            sprintf(OBJ_Existence_Probability_, "OBJ_Existence_Probability_%d", i);
            sprintf(OBJ_Object_Age_, "OBJ_Object_Age_%d", i);
            sprintf(OBJ_Object_Class_, "OBJ_Object_Class_%d", i);
            sprintf(OBJ_Class_Probability_, "OBJ_Class_Probability_%d", i);
            sprintf(OBJ_Car_Probability_, "OBJ_Car_Probability_%d", i);
            sprintf(OBJ_Truck_Probability_, "OBJ_Truck_Probability_%d", i);
            sprintf(OBJ_Bike_Probability_, "OBJ_Bike_Probability_%d", i);
            sprintf(OBJ_Motion_Status_, "OBJ_Motion_Status_%d", i);
            sprintf(OBJ_Has_Cut_Lane_, "OBJ_Has_Cut_Lane_%d", i);
            sprintf(OBJ_Has_Cut_Path_, "OBJ_Has_Cut_Path_%d", i);
            sprintf(OBJ_Lane_Assignment_, "OBJ_Lane_Assignment_%d", i);
            sprintf(OBJ_Long_Distance_, "OBJ_Long_Distance_%d", i);
            sprintf(OBJ_Lat_Distance_, "OBJ_Lat_Distance_%d", i);
            sprintf(OBJ_Relative_Long_Velocity_, "OBJ_Relative_Long_Velocity_%d", i);
            sprintf(OBJ_Relative_Long_Velocity_V_, "OBJ_Relative_Long_Velocity_V_%d", i);
            sprintf(OBJ_Relative_Lat_Velocity_, "OBJ_Relative_Lat_Velocity_%d", i);
            sprintf(OBJ_Relative_Lat_Velocity_V_, "OBJ_Relative_Lat_Velocity_V_%d", i);
            sprintf(OBJ_Width_, "OBJ_Width_%d", i);
            sprintf(OBJ_Length_, "OBJ_Length_%d", i);
            sprintf(OBJ_Heading_, "OBJ_Heading_%d", i);
            sprintf(OBJ_Inv_TTC_, "OBJ_Inv_TTC_%d", i);
            sprintf(OBJ_Inv_TTC_V_, "OBJ_Inv_TTC_V_%d", i);

            if (add_time_offset) /// add time offset only one time
            {
                add_time_offset = false;
                /// set vision_object_detection_data_type
                PyArray_Descr *type = PyArray_DESCR((PyArrayObject *)PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_ID_));
                if (strcmp(type->typeobj->tp_name, "numpy.int32") == 0)
                {
                    vision_object_detection_data_type = INT32;
                    result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_ID), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_ID_),
                                                 add_data_in_object(PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
                }
                else if (strcmp(type->typeobj->tp_name, "numpy.int64") == 0)
                {
                    vision_object_detection_data_type = INT64;
                    result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_ID), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_ID_),
                                                 add_data_in_object(PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
                }
            }

            if (vision_object_detection_data_type == INT32)
            {
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_ID), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_ID_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_Has_Cut_Lane), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Has_Cut_Lane_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_Has_Cut_Path), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Has_Cut_Path_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_Lane_Assignment), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Lane_Assignment_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_Motion_Status), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Motion_Status_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_Object_Age), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Object_Age_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_Object_Class), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Object_Class_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_Relative_Lat_Velocity_V), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Relative_Lat_Velocity_V_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_Relative_Long_Velocity_V), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Relative_Long_Velocity_V_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_object_detection[i].vis_obj_s32.OBJ_Inv_TTC_V), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Inv_TTC_V_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            }
            else if (vision_object_detection_data_type == INT64)
            {
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_ID), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_ID_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_Has_Cut_Lane), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Has_Cut_Lane_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_Has_Cut_Path), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Has_Cut_Path_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_Lane_Assignment), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Lane_Assignment_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_Motion_Status), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Motion_Status_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_Object_Age), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Object_Age_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_Object_Class), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Object_Class_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_Relative_Lat_Velocity_V), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Relative_Lat_Velocity_V_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_Relative_Long_Velocity_V), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Relative_Long_Velocity_V_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_object_detection[i].vis_obj_s64.OBJ_Inv_TTC_V), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Inv_TTC_V_),
                                             PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            }
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Bike_Probability), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Bike_Probability_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Car_Probability), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Car_Probability_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Class_Probability), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Class_Probability_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Existence_Probability), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Existence_Probability_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Height), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Heading_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Inv_TTC), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Inv_TTC_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Lat_Distance), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Lat_Distance_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Length), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Length_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Long_Distance), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Long_Distance_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Relative_Lat_Velocity), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Relative_Lat_Velocity_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Relative_Long_Velocity), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Relative_Long_Velocity_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Truck_Probability), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Truck_Probability_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_object_detection[i].OBJ_Width), PyDict_GetItemWrapper(p_core_objects_protocol, OBJ_Width_),
                                         PyDict_GetItemWrapper(p_core_objects_protocol, "timestamp"));
        }
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_vision_fail_safe(st_pickle_vision_fail_safe_t *p_st_vision_fail_safe, PyObject *const p_core_failsafe_protocol)
{
    bool result = true;
    if (p_core_failsafe_protocol != NULL)
    {
        /// set vision fail safe
        PyArray_Descr *type = PyArray_DESCR((PyArrayObject *)PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Blur_Image_0"));
        if (strcmp(type->typeobj->tp_name, "numpy.int32") == 0)
        {
            vision_fail_safe_data_type = INT32;
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Blur_Image_0, PyDict_GetItemWrapper(p_core_failsafe_protocol,"FS_Blur_Image_0"),
                                        add_data_in_object(PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_C2C_Out_Of_Calib_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_C2C_Out_Of_Calib_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Out_Of_Focus_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Out_Of_Focus_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Calibration_Misalignment, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Calibration_Misalignment"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Fog, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Fog"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Free_Sight_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Free_Sight_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Full_Blockage_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Full_Blockage_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Low_Sun_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Low_Sun_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                                                            
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Partial_Blockage_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Partial_Blockage_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Rain, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Rain"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Splashes_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Splashes_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s32(&p_st_vision_fail_safe->vision_fail_safe_s32.FS_Sun_Ray_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Sun_Ray_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                                                            
        }
        else if (strcmp(type->typeobj->tp_name, "numpy.int64") == 0)
        {
            vision_fail_safe_data_type = INT64;
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Blur_Image_0, PyDict_GetItemWrapper(p_core_failsafe_protocol,"FS_Blur_Image_0"),
                                        add_data_in_object(PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_C2C_Out_Of_Calib_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_C2C_Out_Of_Calib_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Out_Of_Focus_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Out_Of_Focus_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));
            if ( e_pickle_ASTEP != A450_SOP4 ) // f64            
            {
                result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Calibration_Misalignment, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Calibration_Misalignment"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));
            }                                                                        
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Fog, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Fog"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Free_Sight_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Free_Sight_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Full_Blockage_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Full_Blockage_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Low_Sun_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Low_Sun_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                                                            
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Partial_Blockage_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Partial_Blockage_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Rain, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Rain"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Splashes_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Splashes_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                        
            result &= construct_from_s64(&p_st_vision_fail_safe->vision_fail_safe_s64.FS_Sun_Ray_0, PyDict_GetItemWrapper(p_core_failsafe_protocol, "FS_Sun_Ray_0"),
                                        PyDict_GetItemWrapper(p_core_failsafe_protocol, "timestamp"));                                                                                                            
        }
    }                                    
    else
    {
        result = false;
    }
    return result;    
}

static bool parse_vision_free_space(st_pickle_vision_free_space_t *p_st_vision_free_space, PyObject *const p_core_free_space_ext_protocol)
{
    bool result = true;
    if (p_core_free_space_ext_protocol != NULL)
    {
        bool add_time_offset = true;
        char FSPX_Is_Valid_[20];
        char FSPX_Is_Valid_B0_[20]; // 20211015A
        char FSPX_Is_Valid_B0_0[20]; // 20211031A
        char FSPX_Existence_Prob_B0_[30];
        char FSPX_Obstacle_ID_B0_[30];
        char FSPX_Mobility_Status_B0_[30];
        char FSPX_Classification_Type_B0_[30];
        char FSPX_Range_B0_[20];
        char FSPX_Range_STD_B0_[30];
        char FSPX_Height_B0_[20];
        char FSPX_Height_STD_B0_[30];
        char FSPX_Azimuth_Angle_B0_[30];
        char FSPX_Lane_Assignment_B0_[30];
        for (int i = 0; i < CORE_FREE_SPACE_NUM_OF_ELEMENTS; i++)
        {
            /// sprintf(FSPX_Is_Valid_, "FSPX_Is_Valid_%d", i); /// 20211015A: in core protocol 4.21.5, _B0 added
            sprintf(FSPX_Is_Valid_, "FSPX_Is_Valid_%d", i); // 20211015A: in core protocol 4.21.5, _B0 added
            sprintf(FSPX_Is_Valid_B0_, "FSPX_Is_Valid_B0_%d", i); // 20211015A: in core protocol 4.21.5, _B0 added
            sprintf(FSPX_Is_Valid_B0_0, "FSPX_Is_Valid_B0_0%d", i); // 20211031A: in core protocol for A450_SOP4, _B0_0 added
            sprintf(FSPX_Existence_Prob_B0_, "FSPX_Existence_Prob_B0_%d", i);
            sprintf(FSPX_Obstacle_ID_B0_, "FSPX_Obstacle_ID_B0_%d", i);
            sprintf(FSPX_Mobility_Status_B0_, "FSPX_Mobility_Status_B0_%d", i);
            sprintf(FSPX_Classification_Type_B0_, "FSPX_Classification_Type_B0_%d", i);
            sprintf(FSPX_Range_B0_, "FSPX_Range_B0_%d", i);
            sprintf(FSPX_Range_STD_B0_, "FSPX_Range_STD_B0_%d", i);
            sprintf(FSPX_Height_B0_, "FSPX_Height_B0_%d", i);
            sprintf(FSPX_Height_STD_B0_, "FSPX_Height_STD_B0_%d", i);
            sprintf(FSPX_Azimuth_Angle_B0_, "FSPX_Azimuth_Angle_B0_%d", i);
            sprintf(FSPX_Lane_Assignment_B0_, "FSPX_Lane_Assginment_B0_%d", i);
            if (add_time_offset) /// add time offset only one time
            {
                add_time_offset = false;
                /// set vision_free_space_data_type
                PyArray_Descr *type = PyArray_DESCR((PyArrayObject *)PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Obstacle_ID_B0_));
                if (strcmp(type->typeobj->tp_name, "numpy.int32") == 0)
                {
                    vision_free_space_data_type = INT32;
                    result &= construct_from_s32(&(p_st_vision_free_space[i].vision_free_space_s32.FSP_ID), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Obstacle_ID_B0_),
                                                 add_data_in_object(PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
                }
                else if (strcmp(type->typeobj->tp_name, "numpy.int64") == 0)
                {
                    vision_free_space_data_type = INT64;
                    result &= construct_from_s64(&(p_st_vision_free_space[i].vision_free_space_s64.FSP_ID), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Obstacle_ID_B0_),
                                                 add_data_in_object(PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
                }
            }

            if (vision_free_space_data_type == INT32)
            {
                bool result_FSP_is_valid = false ; // 20211015A
                result &= construct_from_s32(&(p_st_vision_free_space[i].vision_free_space_s32.FSP_ID), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Obstacle_ID_B0_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));

                result_FSP_is_valid = construct_from_s32(&(p_st_vision_free_space[i].vision_free_space_s32.FSP_Is_Valid), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Is_Valid_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp")); // 20211015A
                if ( result_FSP_is_valid == false ) { // 20211015A
                    result_FSP_is_valid = construct_from_s32(&(p_st_vision_free_space[i].vision_free_space_s32.FSP_Is_Valid), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Is_Valid_B0_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp")); // 20211015A
                }
                if ( result_FSP_is_valid == false ) { // 20211031A
                    result_FSP_is_valid = construct_from_s32(&(p_st_vision_free_space[i].vision_free_space_s32.FSP_Is_Valid), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Is_Valid_B0_0),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp")); // 20211031A
                }                
                result &= result_FSP_is_valid ; // 20211015A

                result &= construct_from_s32(&(p_st_vision_free_space[i].vision_free_space_s32.FSP_Mobility_Status), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Mobility_Status_B0_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_free_space[i].vision_free_space_s32.FSP_Classification_Type), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Classification_Type_B0_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));
                result &= construct_from_s32(&(p_st_vision_free_space[i].vision_free_space_s32.FSP_Lane_Assignment), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Lane_Assignment_B0_),
                                            PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));                                                     
            }
            else if (vision_free_space_data_type == INT64)
            {
                bool result_FSP_is_valid = false ; // 20211031A
                
                result &= construct_from_s64(&(p_st_vision_free_space[i].vision_free_space_s64.FSP_ID), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Obstacle_ID_B0_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));

                result_FSP_is_valid = construct_from_s64(&(p_st_vision_free_space[i].vision_free_space_s64.FSP_Is_Valid), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Is_Valid_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));

                if ( result_FSP_is_valid == false ) { // 20211015A
                    result_FSP_is_valid = construct_from_s64(&(p_st_vision_free_space[i].vision_free_space_s64.FSP_Is_Valid), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Is_Valid_B0_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp")); // 20211015A
                }
                if ( result_FSP_is_valid == false ) { // 20211031A
                    result_FSP_is_valid = construct_from_s64(&(p_st_vision_free_space[i].vision_free_space_s64.FSP_Is_Valid), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Is_Valid_B0_0),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp")); // 20211031A
                } 
                result &= result_FSP_is_valid ; // 20211031A
                result &= construct_from_s64(&(p_st_vision_free_space[i].vision_free_space_s64.FSP_Mobility_Status), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Mobility_Status_B0_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_free_space[i].vision_free_space_s64.FSP_Classification_Type), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Classification_Type_B0_),
                                             PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));
                result &= construct_from_s64(&(p_st_vision_free_space[i].vision_free_space_s64.FSP_Lane_Assignment), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Lane_Assignment_B0_),
                                            PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));                                                     
            }

            result &= construct_from_f64(&(p_st_vision_free_space[i].FSP_Existence_Prob), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Existence_Prob_B0_),
                                         PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_free_space[i].FSP_Height), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Height_B0_),
                                         PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_free_space[i].FSP_Height_STD), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Height_STD_B0_),
                                         PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_free_space[i].FSP_Range), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Range_B0_),
                                         PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_free_space[i].FSP_Range_STD), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Range_STD_B0_),
                                         PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));
            result &= construct_from_f64(&(p_st_vision_free_space[i].FSP_Azimuth_Angle), PyDict_GetItemWrapper(p_core_free_space_ext_protocol, FSPX_Azimuth_Angle_B0_),
                                         PyDict_GetItemWrapper(p_core_free_space_ext_protocol, "timestamp"));                                 
        }
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_vision_lane_support_road_edge(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_road_edge)
{
    bool result = true;
    if (p_spi_eyeq_core_lanes_road_edge != NULL)
    {
        (void)parse_vision_lane_support_road_edge_left_first(p_st_road_edge, p_spi_eyeq_core_lanes_road_edge);
        (void)parse_vision_lane_support_road_edge_right_first(p_st_road_edge, p_spi_eyeq_core_lanes_road_edge);
        /// commented out in import_pickle_data_versions.m
        // result &= parse_vision_lane_support_road_edge_left_second(p_st_road_edge, p_spi_eyeq_core_lanes_road_edge);
        // result &= parse_vision_lane_support_road_edge_right_second(p_st_road_edge, p_spi_eyeq_core_lanes_road_edge);
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_vision_lane_support_road_edge_left_first(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_road_edge)
{
    bool result = true;
    if (p_spi_eyeq_core_lanes_road_edge != NULL)
    {
        /// set vision_lane_support_road_edge_data_type
        PyArray_Descr *type = PyArray_DESCR((PyArrayObject *)PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_ID_0"));
        if (strcmp(type->typeobj->tp_name, "numpy.int32") == 0)
        {
            vision_lane_support_road_edge_data_type = INT32;
        }
        else if (strcmp(type->typeobj->tp_name, "numpy.int64") == 0)
        {
            vision_lane_support_road_edge_data_type = INT64;
        }
        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_left_first.vision_road_edge_s64.LS_Road_Edge_ID,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_ID_0"),
                                         add_data_in_object(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_left_first.vision_road_edge_s32.LS_Road_Edge_ID,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_ID_0"),
                                         add_data_in_object(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"), FLOAT64, -1 * f64_time_offset_g));
        }

        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_left_first.vision_road_edge_s64.LS_Road_Edge_Availability_State,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Availability_State_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_left_first.vision_road_edge_s32.LS_Road_Edge_Availability_State,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Availability_State_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }        

        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_left_first.vision_road_edge_s64.LS_Road_Edge_Prediction_Reason,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_left_first.vision_road_edge_s32.LS_Road_Edge_Prediction_Reason,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }        


        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_left_first.vision_road_edge_s64.LS_Road_Edge_Side,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Side_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_left_first.vision_road_edge_s32.LS_Road_Edge_Side,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Side_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }

        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_left_first.vision_road_edge_s64.LS_Road_Edge_Age,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Age_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_left_first.vision_road_edge_s32.LS_Road_Edge_Age,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Age_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }

        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_left_first.vision_road_edge_s64.LS_Prediction_Type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
            result &= construct_from_s64(&p_st_road_edge->edge_left_first.vision_road_edge_s64.LS_Road_Edge_Type_Avg_Class,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Type_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_left_first.vision_road_edge_s32.LS_Prediction_Type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
            result &= construct_from_s32(&p_st_road_edge->edge_left_first.vision_road_edge_s32.LS_Road_Edge_Type_Avg_Class,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Type_0"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }

        result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Exist_Prob,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Confidence_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Height,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Height_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_View_Range_Start,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_View_Range_Start_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_View_Range_End,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_View_Range_End_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        /// @TODO: All coding parameters are read as zeros now since they are initialized with values at
        ///        later point of time in code. More investigation is needed to see how to read these parameters
        ///        with correct values. For now we only use codingAllInput.vehicleCoding.C_CAMERA_LAT_POS which is
        ///        initialized with 0
        // subtract camera lat pos coding param
        result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C0,
                                     add_data_in_object(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C0_0"), FLOAT64, codingAllInput.vehicleCoding.C_CAMERA_LAT_POS),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C0_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C0_0"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C1,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C1_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C1_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C1_0"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C2,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C2_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C2_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C2_0"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C3,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C3_0"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C3_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C3_0"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        assign_f64_pickle_struct(&(p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C0_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C1_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C2_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(p_st_road_edge->edge_left_first.LS_Road_Edge_Line_C3_STD), &st_lca_pickle_base_f64_g);
    }
    else
    {
        result = false;
    }
    return result;
}

static bool parse_vision_lane_support_road_edge_right_first(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_road_edge)
{
    ///@TODO: If needed, then Subtract f64_time_offset_g offset
    bool result = true;
    if (p_spi_eyeq_core_lanes_road_edge != NULL)
    {
        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_right_first.vision_road_edge_s64.LS_Road_Edge_ID,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_ID_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_right_first.vision_road_edge_s32.LS_Road_Edge_ID,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_ID_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }

        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_right_first.vision_road_edge_s64.LS_Road_Edge_Availability_State,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Availability_State_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_right_first.vision_road_edge_s32.LS_Road_Edge_Availability_State,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Availability_State_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }        

        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_right_first.vision_road_edge_s64.LS_Road_Edge_Prediction_Reason,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_right_first.vision_road_edge_s32.LS_Road_Edge_Prediction_Reason,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }        

        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_right_first.vision_road_edge_s64.LS_Road_Edge_Side,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Side_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_right_first.vision_road_edge_s32.LS_Road_Edge_Side,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Side_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }

        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_right_first.vision_road_edge_s64.LS_Road_Edge_Age,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Age_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_right_first.vision_road_edge_s32.LS_Road_Edge_Age,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Age_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }

        if (vision_lane_support_road_edge_data_type == INT64)
        {
            result &= construct_from_s64(&p_st_road_edge->edge_right_first.vision_road_edge_s64.LS_Prediction_Type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
            result &= construct_from_s64(&p_st_road_edge->edge_right_first.vision_road_edge_s64.LS_Road_Edge_Type_Avg_Class,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Type_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }
        else if (vision_lane_support_road_edge_data_type == INT32)
        {
            result &= construct_from_s32(&p_st_road_edge->edge_right_first.vision_road_edge_s32.LS_Prediction_Type,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
            result &= construct_from_s32(&p_st_road_edge->edge_right_first.vision_road_edge_s32.LS_Road_Edge_Type_Avg_Class,
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Type_1"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        }

        result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Exist_Prob,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Confidence_1"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Height,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Height_1"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_View_Range_Start,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_View_Range_Start_1"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_View_Range_End,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_View_Range_End_1"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // subtract camera lat pos coding param
        result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C0,
                                     add_data_in_object(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C0_1"), FLOAT64, codingAllInput.vehicleCoding.C_CAMERA_LAT_POS),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C0_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C1_0"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C1,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C1_1"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C1_STD,
        //Set to zero through the common zero pickle_struct
        //  scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C1_1"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C2,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C2_1"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C2_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C2_1"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C3,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C3_1"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C3_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C3_1"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        assign_f64_pickle_struct(&(p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C0_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C1_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C2_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(p_st_road_edge->edge_right_first.LS_Road_Edge_Line_C3_STD), &st_lca_pickle_base_f64_g);
    }
    else
    {
        result = false;
    }
    return result;
}

/*
static bool parse_vision_lane_support_road_edge_left_second(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_road_edge)
{
    ///@TODO: If needed, then Subtract f64_time_offset_g offset
    bool result = true;
    if (p_spi_eyeq_core_lanes_road_edge != NULL)
    {
        result &= construct_from_s64(&p_st_road_edge->edge_left_second.LS_Road_Edge_ID,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_ID_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_s64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Age,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Age_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Exist_Prob,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Confidence_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        #ifdef Type_Of_Prediction_Type_INT32
            result &= construct_from_s32(&p_st_road_edge->edge_left_second.LS_Prediction_Type,
                                        PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_2"),
                                         PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        #elif defined Type_Of_Prediction_Type_INT64
            result &= construct_from_s64(&p_st_road_edge->edge_left_second.LS_Prediction_Type,
                                        PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_2"),
                                        PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        #endif

        result &= construct_from_s64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Type_Avg_Class,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Type_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Height,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Height_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_View_Range_Start,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_View_Range_Start_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_View_Range_End,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_View_Range_End_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // subtraction
        result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C0,
                                     add_data_in_object(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C0_2"), FLOAT64, codingAllInput.vehicleCoding.C_CAMERA_LAT_POS),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C0_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C0_2"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C1,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C1_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C1_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C1_2"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C2,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C2_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C2_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C2_2"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C3,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C3_2"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C3_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C3_2"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        assign_f64_pickle_struct(&(p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C0_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C1_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C2_STD), &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&(p_st_road_edge->edge_left_second.LS_Road_Edge_Line_C3_STD), &st_lca_pickle_base_f64_g);
    }
    else
    {
        result = false;
    }
    return result;
}
*/
/*
static bool parse_vision_lane_support_road_edge_right_second(st_pickle_vision_lane_support_road_edge_t *p_st_road_edge, PyObject *const p_spi_eyeq_core_lanes_road_edge)
{
    ///@TODO: If needed, then Subtract f64_time_offset_g offset
    bool result = true;
    if (p_spi_eyeq_core_lanes_road_edge != NULL)
    {
        //TODO Possible bug in Matlab script
        result &= construct_from_s64(&p_st_road_edge->edge_right_second.LS_Road_Edge_ID,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_ID_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_s64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Age,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Age_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Exist_Prob,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Confidence_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        #ifdef Type_Of_Prediction_Type_INT32
        result &= construct_from_s32(&p_st_road_edge->edge_right_second.LS_Prediction_Type,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        #elif defined Type_Of_Prediction_Type_INT64
        result &= construct_from_s64(&p_st_road_edge->edge_right_second.LS_Prediction_Type,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Prediction_Reason_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));
        #endif

        result &= construct_from_s64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Type_Avg_Class,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Type_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Height,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Height_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_View_Range_Start,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_View_Range_Start_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_View_Range_End,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_View_Range_End_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        //subtraction
        result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C0,
                                     add_data_in_object(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C0_3"), FLOAT64, codingAllInput.vehicleCoding.C_CAMERA_LAT_POS),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C0_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C0_3"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C1,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C1_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C1_STD,
        //Set to zero through the common zero pickle_struct
        //  scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C1_3"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C2,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C2_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C2_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C2_3"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C3,
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C3_3"),
                                     PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        // result &= construct_from_f64(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C3_STD,
        //Set to zero through the common zero pickle_struct
        // scale_data(PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "LRE_Line_C3_3"), FLOAT64, 0.0),
        // PyDict_GetItemWrapper(p_spi_eyeq_core_lanes_road_edge, "timestamp"));

        assign_f64_pickle_struct(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C0_STD, &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C1_STD, &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C2_STD, &st_lca_pickle_base_f64_g);
        assign_f64_pickle_struct(&p_st_road_edge->edge_right_second.LS_Road_Edge_Line_C3_STD, &st_lca_pickle_base_f64_g);
    }
    else
    {
        result = false;
    }
    return result;
}
*/

static bool parse_bcm_signal(st_pickle_bcm_signal_t *const p_st_bcm_signal, PyObject *const p_sp_2021)
{
    bool result = true;
    bool r[11] ; // 20211031A
    if (p_sp_2021 != NULL)
    {
        /// Mass vehicle and status trailer got removed, angle accelerator pedal ///
        r[0] = construct_from_f64(&p_st_bcm_signal->vehicle_mass_f64,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "VehicleModel"),
                                            "MassVehicle2"),
                                            "massVeh"),
                                    add_data_in_object(PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "VehicleModel"),
                                            "MassVehicle2"),
                                        "timestamp"),
                                        FLOAT64, -1 * f64_time_offset_g));

        /* controlIndicateDirection */
        r[1] = construct_from_u8(&p_st_bcm_signal->activeFunctionIndicateDirectionStruct.controlIndicateDirection,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "Light2"),
                                            "statusFunctionsIndicatorsEth"),
                                        "controlIndicateDirection"),
                                    add_data_in_object(PyDict_GetItemWrapper(
                                                           PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(p_sp_2021, "Light2"),
                                                               "statusFunctionsIndicatorsEth"),
                                                           "timestamp"),
                                                       FLOAT64, -1 * f64_time_offset_g));

        r[2] = construct_from_u8(&p_st_bcm_signal->activeFunctionIndicateDirectionStruct.activeFunctionIndicateDirection,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "Light2"),
                                            "statusFunctionsIndicatorsEth"),
                                        "activeFunctionIndicateDirection"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "Light2"),
                                            "statusFunctionsIndicatorsEth"),
                                        "timestamp"));

        /* steeringAngleDriver */
        if ( e_pickle_ASTEP == A450_SOP4 ) // for A450_SOP4, get from EgoMotionSignalProvision
        {
            r[3] = construct_from_f64(&p_st_bcm_signal->steeringAngleDriver.steeringAngleDriver_f64,
                                         scale_data(PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                            "egoMotionSignal"),
                                                        "steeringAngleDriver"),
                                                    FLOAT64, NEGATIVE_CONV),
                                        PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"),
                                                                    "egoMotionSignal"),
                                                                "timestamp"));
	    }
	    else
	    {
            r[3] = construct_from_f32(&p_st_bcm_signal->steeringAngleDriver.steeringAngleDriver,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "SteeringAngleProvision"),
                                                            "steeringAngleDriver"),
                                                        "steeringAngleDriver"),
                                                    "steeringAngleDriver"),
                                                FLOAT32, NEGATIVE_CONV),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "SteeringAngleProvision"),
                                                                "steeringAngleDriver"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
	    }
        /* angleAcceleratorPedal */
        r[4] = construct_from_f32(&p_st_bcm_signal->angleAcceleratorPedal.actualValueAngleAcceleratorPedal,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "AngleAcceleratorPedal2"),
                                                            "angleAcceleratorPedal"),
                                                        "actualValueAngleAcceleratorPedal"),
                                                    "actualValueAngleAcceleratorPedal"),
                                                FLOAT32, BCM_ANGLE_ACCEL_MULTI_CONSTANT),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "AngleAcceleratorPedal2"),
                                                                "angleAcceleratorPedal"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));

        r[5] = construct_from_f64(&p_st_bcm_signal->angleAcceleratorPedal.gradientActualValueAngleAcceleratorPedal,
                                     add_data_in_object(scale_data(PyDict_GetItemWrapper(
                                                                       PyDict_GetItemWrapper(
                                                                           PyDict_GetItemWrapper(
                                                                               PyDict_GetItemWrapper(p_sp_2021, "AngleAcceleratorPedal2"),
                                                                               "angleAcceleratorPedal"),
                                                                           "gradientActualValueAngleAcceleratorPedal"),
                                                                       "gradientActualValueAngleAcceleratorPedal"),
                                                                   FLOAT64, BCM_ANGLE_GRADIENT_MULTI_CONSTANT),
                                                        FLOAT64, -1 * BCM_ANGLE_GRADIENT_SUB_CONSTANT),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "AngleAcceleratorPedal2"),
                                             "angleAcceleratorPedal"),
                                         "timestamp"));

        /* actualValueBrakingTorqueSum */
        r[6] = construct_from_s32(&p_st_bcm_signal->actualBrakingTorqueProvision.actualValueBrakingTorqueSum,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "TargetBrakingTorqueDriverProvision"),
                                                 "targetBrakingTorqueDriverProvision"),
                                             "actualValueBrakingTorqueSumDriversChoice"),
                                         "actualValueBrakingTorqueSumDriversChoice"),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "TargetBrakingTorqueDriverProvision"),
                                                                "targetBrakingTorqueDriverProvision"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
        /* actualValueSteeringMomentDriverActuator */
        r[7] = construct_from_f32(&p_st_bcm_signal->handwheelTorqueProvision.actualValueSteeringMomentDriverActuator,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "HandwheelTorqueProvision"),
                                                            "handwheelTorqueProvision"),
                                                        "actualValueSteeringMomentDriverActuator"),
                                                    "actualValueSteeringMomentDriverActuator"),
                                                FLOAT32, NEGATIVE_CONV),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "HandwheelTorqueProvision"),
                                                                "handwheelTorqueProvision"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
        /* actualValueSteeringMomentDriverActuator */
        r[8] = construct_from_u8(&p_st_bcm_signal->statusBrakingDriver.statusBrakingDriver,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "TargetBrakingTorqueDriverProvision"),
                                                "statusBrakingDriverProvision"),
                                            "statusBrakingDriver"),
                                        "statusBrakingDriver"),
                                    add_data_in_object(PyDict_GetItemWrapper(
                                                           PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(p_sp_2021, "TargetBrakingTorqueDriverProvision"),
                                                               "statusBrakingDriverProvision"),
                                                           "timestamp"),
                                                       FLOAT64, -1 * f64_time_offset_g));
        /* statusDynamometer */
        r[9] = construct_from_u8(&p_st_bcm_signal->StatusDynamometerAndEndOfAssemblyLine.statusDynamometer,
                          PyDict_GetItemWrapper(
                              PyDict_GetItemWrapper(
                                  PyDict_GetItemWrapper(p_sp_2021, "DynamometerAndEndOfAssemblyLineProvision"),
                                  "statusDynamometerAndEndOfAssemblyLine"),
                              "statusDynamometer"),
                          add_data_in_object(PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(p_sp_2021, "DynamometerAndEndOfAssemblyLineProvision"),
                                                     "statusDynamometerAndEndOfAssemblyLine"),
                                                 "timestamp"),
                                             FLOAT64, -1 * f64_time_offset_g));
        /* statusEndOfAssemblyLine */
        r[10] = construct_from_u8(&p_st_bcm_signal->StatusDynamometerAndEndOfAssemblyLine.statusEndOfAssemblyLine,
                          PyDict_GetItemWrapper(
                              PyDict_GetItemWrapper(
                                  PyDict_GetItemWrapper(p_sp_2021, "DynamometerAndEndOfAssemblyLineProvision"),
                                  "statusDynamometerAndEndOfAssemblyLine"),
                              "statusEndOfAssemblyLine"),
                          PyDict_GetItemWrapper(
                              PyDict_GetItemWrapper(
                                  PyDict_GetItemWrapper(p_sp_2021, "DynamometerAndEndOfAssemblyLineProvision"),
                                  "statusDynamometerAndEndOfAssemblyLine"),
                              "timestamp"));
        result = true ;
        for (int n = 0 ; n <= 10; n++){
            result &= r[n] ;
        }
    }
    else
    {
        result = false;
    }

    return result;
}

static bool parse_srr_scw_object_signal(st_pickle_srr_scw_object_t *const p_st_srr_scw_object, PyObject *const p_sp_2021)
{
    bool result = true;
    if (p_sp_2021 != NULL)
    {
        /* criticalObjectLeftAX */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftAX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectLeftAX"),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                "criticalObjectSideCollision"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
        /* criticalObjectLeftAY */ // multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftAY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectSideCollision"),
                                                        "sideCollisionCriticalObjectInfo"),
                                                    "criticalObjectLeftAY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectLeftExistenceProbability */
        result &= construct_from_u16(&p_st_srr_scw_object->criticalObjectLeftExistenceProbability,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectLeftExistenceProbability"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectLeftHeading */ // multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftHeading,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectSideCollision"),
                                                        "sideCollisionCriticalObjectInfo"),
                                                    "criticalObjectLeftHeading"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectLeftID */
        result &= construct_from_u8(&p_st_srr_scw_object->criticalObjectLeftID,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                "criticalObjectSideCollision"),
                                            "sideCollisionCriticalObjectInfo"),
                                        "criticalObjectLeftID"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectSideCollision"),
                                        "timestamp"));
        /* criticalObjectLeftLength */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftLength,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectLeftLength"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectLeftPOX */ // minus codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftPOX,
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                    "criticalObjectSideCollision"),
                                                                "sideCollisionCriticalObjectInfo"),
                                                            "criticalObjectLeftPOX"),
                                                        FLOAT32, -1. * codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectLeftPOY */ // Multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftPOY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectSideCollision"),
                                                        "sideCollisionCriticalObjectInfo"),
                                                    "criticalObjectLeftPOY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectLeftTTCX */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftTTCX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectLeftTTCX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectLeftVX */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftVX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectLeftVX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectLeftVY */ // multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftVY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectSideCollision"),
                                                        "sideCollisionCriticalObjectInfo"),
                                                    "criticalObjectLeftVY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectLeftWidth */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftWidth,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectLeftWidth"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightAge */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectLeftAge,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectLeftAge"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));                                         
        /* criticalObjectRightAX */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightAX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectRightAX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightAY */ // multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightAY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectSideCollision"),
                                                        "sideCollisionCriticalObjectInfo"),
                                                    "criticalObjectRightAY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightExistenceProbability */
        result &= construct_from_u16(&p_st_srr_scw_object->criticalObjectRightExistenceProbability,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectRightExistenceProbability"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightHeading */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightHeading,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectSideCollision"),
                                                        "sideCollisionCriticalObjectInfo"),
                                                    "criticalObjectRightHeading"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightID */
        result &= construct_from_u8(&p_st_srr_scw_object->criticalObjectRightID,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                "criticalObjectSideCollision"),
                                            "sideCollisionCriticalObjectInfo"),
                                        "criticalObjectRightID"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectSideCollision"),
                                        "timestamp"));
        /* criticalObjectRightLength */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightLength,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectRightLength"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightPOX */ // minus codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightPOX,
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                    "criticalObjectSideCollision"),
                                                                "sideCollisionCriticalObjectInfo"),
                                                            "criticalObjectRightPOX"),
                                                        FLOAT32, -1. * codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightPOY */ // Multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightPOY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectSideCollision"),
                                                        "sideCollisionCriticalObjectInfo"),
                                                    "criticalObjectRightPOY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightTTCX */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightTTCX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectRightTTCX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightVX */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightVX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectRightVX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightVY */ // multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightVY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectSideCollision"),
                                                        "sideCollisionCriticalObjectInfo"),
                                                    "criticalObjectRightVY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightWidth */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightWidth,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectRightWidth"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));
        /* criticalObjectRightAge */
        result &= construct_from_f32(&p_st_srr_scw_object->criticalObjectRightAge,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectSideCollision"),
                                             "sideCollisionCriticalObjectInfo"),
                                         "criticalObjectRightAge"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectSideCollision"),
                                         "timestamp"));

        /* statusGuardrail */
        result &= construct_from_u8(&p_st_srr_scw_object->statusGuardrail,
                                    replace_data_in_object(PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                   "criticalObjectSideCollision"),
                                                               "statusGuardrail"),
                                                           UINT8, 1.0, 0.0),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectSideCollision"),
                                        "timestamp"));
        /* StatusSideCollisionWarning */
        result &= construct_from_u8(&p_st_srr_scw_object->statusSideCollisionWarning,
                                    replace_data_in_object(PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                   "criticalObjectSideCollision"),
                                                               "statusSideCollisionWarning"),
                                                           UINT8, 1.0, 0.0),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectSideCollision"),
                                        "timestamp"));
    }
    else
    {
        result = false;
    }

    return result;
}

static bool parse_frr_object_lists_signal(st_pickle_frr_object_list_t *const p_st_frr_object_list, PyObject *const p_sp_2021)
{
    bool result = true;

    if (p_sp_2021 != NULL)
    {
        char index[3];
        for (int i = 0; i < FRR_OBJECT_LIST_SIZE; i++)
        {
            sprintf(index, "%d", i);
            if (i == 0)
            {
                result &= construct_from_u16(&(p_st_frr_object_list[i].id),
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "summary"),
                                                 "id"),
                                             add_data_in_object(PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(
                                                                        PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                        "Objectlist"),
                                                                    "timestamp"),
                                                                FLOAT64, -1 * f64_time_offset_g));
            }
            else
            {
                result &= construct_from_u16(&(p_st_frr_object_list[i].id),
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "summary"),
                                                 "id"),
                                             PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                       PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                       "Objectlist"),
                                                                   "timestamp"));
            }
            g_toggle_printing_warning_messages = false;
            result &= construct_from_u16(&(p_st_frr_object_list[i].age),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                             "Objectlist"),
                                                         "object_list"),
                                                     index),
                                                 "summary"),
                                             "age"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].status_measurement),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                            "Objectlist"),
                                                        "object_list"),
                                                    index),
                                                "summary"),
                                            "status_measurement"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].status_movement),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                            "Objectlist"),
                                                        "object_list"),
                                                    index),
                                                "summary"),
                                            "status_movement"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].existence_ppv),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                            "Objectlist"),
                                                        "object_list"),
                                                    index),
                                                "existence"),
                                            "existence_ppv"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].existence_probability),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                            "Objectlist"),
                                                        "object_list"),
                                                    index),
                                                "existence"),
                                            "existence_probability"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].reference_point),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                            "Objectlist"),
                                                        "object_list"),
                                                    index),
                                                "position"),
                                            "reference_point"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].position_x),
                                         add_data_in_object(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "position"),
                                                 "x"),
                                             "EmMeterRangeXY"),FLOAT64, -1. * codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].position_x_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "position"),
                                                 "x_std_dev"),
                                             "EmMeterRangeXY"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f32(&(p_st_frr_object_list[i].position_covariance_xy),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                             "Objectlist"),
                                                         "object_list"),
                                                     index),
                                                 "position"),
                                             "covariance_xy"),FLOAT32, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].position_y),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "position"),
                                                 "y"),
                                             "EmMeterRangeXY"),FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].position_y_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "position"),
                                                 "y_std_dev"),
                                             "EmMeterRangeXY"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].orientation),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "position"),
                                                 "orientation"),
                                             "EmRadianOrientation"),FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].orientation_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "position"),
                                                 "orientation_std_dev"),
                                             "EmRadianOrientation"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));

            result &= construct_from_f64(&(p_st_frr_object_list[i].velocity_absolute_x),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "velocity_absolute"),
                                                 "x"),
                                             "EmMeterVelocityV"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].velocity_absolute_x_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "velocity_absolute"),
                                                 "x_std_dev"),
                                             "EmMeterVelocityV"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f32(&(p_st_frr_object_list[i].velocity_absolute_covariance_xy),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "dynamics"),
                                                 "velocity_absolute"),
                                             "covariance_xy"),FLOAT32, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].velocity_absolute_y),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "velocity_absolute"),
                                                 "y"),
                                             "EmMeterVelocityV"),FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].velocity_absolute_y_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "velocity_absolute"),
                                                 "y_std_dev"),
                                             "EmMeterVelocityV"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));

            result &= construct_from_f64(&(p_st_frr_object_list[i].velocity_relative_x),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "velocity_relative"),
                                                 "x"),
                                             "EmMeterVelocityV"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].velocity_relative_x_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "velocity_relative"),
                                                 "x_std_dev"),
                                             "EmMeterVelocityV"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f32(&(p_st_frr_object_list[i].velocity_relative_covariance_xy),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "dynamics"),
                                                 "velocity_relative"),
                                             "covariance_xy"),FLOAT32, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].velocity_relative_y),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "velocity_relative"),
                                                 "y"),
                                             "EmMeterVelocityV"),FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].velocity_relative_y_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "velocity_relative"),
                                                 "y_std_dev"),
                                             "EmMeterVelocityV"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));

            result &= construct_from_f64(&(p_st_frr_object_list[i].acceleration_absolute_x),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "acceleration_absolute"),
                                                 "x"),
                                             "EmMeterAccelerationA"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].acceleration_absolute_x_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "acceleration_absolute"),
                                                 "x_std_dev"),
                                             "EmMeterAccelerationA"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f32(&(p_st_frr_object_list[i].acceleration_absolute_covariance_xy),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "dynamics"),
                                                 "acceleration_absolute"),
                                             "covariance_xy"),FLOAT32, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].acceleration_absolute_y),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "acceleration_absolute"),
                                                 "y"),
                                             "EmMeterAccelerationA"),FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].acceleration_absolute_y_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "acceleration_absolute"),
                                                 "y_std_dev"),
                                             "EmMeterAccelerationA"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));

            result &= construct_from_f64(&(p_st_frr_object_list[i].acceleration_relative_x),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "acceleration_relative"),
                                                 "x"),
                                             "EmMeterAccelerationA"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].acceleration_relative_x_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "acceleration_relative"),
                                                 "x_std_dev"),
                                             "EmMeterAccelerationA"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f32(&(p_st_frr_object_list[i].acceleration_relative_covariance_xy),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "dynamics"),
                                                 "acceleration_relative"),
                                             "covariance_xy"),FLOAT32, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].acceleration_relative_y),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "acceleration_relative"),
                                                 "y"),
                                             "EmMeterAccelerationA"),FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].acceleration_relative_y_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "acceleration_relative"),
                                                 "y_std_dev"),
                                             "EmMeterAccelerationA"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));

            result &= construct_from_f64(&(p_st_frr_object_list[i].orientation_rate_mean),
                                         scale_data(PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "orientation_rate"),
                                                 "mean"),
                                             "EmRadianYawrate"),FLOAT64, NEGATIVE_CONV),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].orientation_rate_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "dynamics"),
                                                     "orientation_rate"),
                                                 "std_dev"),
                                             "EmRadianYawrate"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));

            result &= construct_from_f64(&(p_st_frr_object_list[i].shape_length_edge_mean),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "shape"),
                                                     "length"),
                                                 "mean"),
                                             "EmMeterRangeXY"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].shape_length_edge_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "shape"),
                                                     "length"),
                                                 "std_dev"),
                                             "EmMeterRangeXY"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_u32(&(p_st_frr_object_list[i].shape_length_status),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "shape"),
                                                 "length"),
                                             "status"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));

            result &= construct_from_f64(&(p_st_frr_object_list[i].shape_width_edge_mean),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "shape"),
                                                     "width"),
                                                 "mean"),
                                             "EmMeterRangeXY"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_f64(&(p_st_frr_object_list[i].shape_width_edge_std_dev),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(
                                                                     PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                     "Objectlist"),
                                                                 "object_list"),
                                                             index),
                                                         "shape"),
                                                     "width"),
                                                 "std_dev"),
                                             "EmMeterRangeXY"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));
            result &= construct_from_u32(&(p_st_frr_object_list[i].shape_width_status),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(
                                                             PyDict_GetItemWrapper(
                                                                 PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                 "Objectlist"),
                                                             "object_list"),
                                                         index),
                                                     "shape"),
                                                 "width"),
                                             "status"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                   "Objectlist"),
                                                               "timestamp"));

            result &= construct_from_u8(&(p_st_frr_object_list[i].classification_class_car),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                "Objectlist"),
                                                            "object_list"),
                                                        index),
                                                    "classification"),
                                                "class_car"),
                                            "EmPercent"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].classification_class_truck),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                "Objectlist"),
                                                            "object_list"),
                                                        index),
                                                    "classification"),
                                                "class_truck"),
                                            "EmPercent"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].classification_class_motorcycle),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                "Objectlist"),
                                                            "object_list"),
                                                        index),
                                                    "classification"),
                                                "class_motorcycle"),
                                            "EmPercent"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].classification_class_bicycle),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                "Objectlist"),
                                                            "object_list"),
                                                        index),
                                                    "classification"),
                                                "class_bicycle"),
                                            "EmPercent"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].classification_class_pedestrian),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                "Objectlist"),
                                                            "object_list"),
                                                        index),
                                                    "classification"),
                                                "class_pedestrian"),
                                            "EmPercent"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].classification_class_animal),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                "Objectlist"),
                                                            "object_list"),
                                                        index),
                                                    "classification"),
                                                "class_animal"),
                                            "EmPercent"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].classification_class_hazard),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                "Objectlist"),
                                                            "object_list"),
                                                        index),
                                                    "classification"),
                                                "class_hazard"),
                                            "EmPercent"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            result &= construct_from_u8(&(p_st_frr_object_list[i].classification_class_unknown),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                "Objectlist"),
                                                            "object_list"),
                                                        index),
                                                    "classification"),
                                                "class_unknown"),
                                            "EmPercent"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "RecogFullRangeRadarObjectlist"),
                                                                  "Objectlist"),
                                                              "timestamp"));
            g_toggle_printing_warning_messages = true;
        }
    }
    else
    {
        result = false;
    }

    return result;
}

static bool parse_srr_free_space_signal(st_pickle_srr_free_space_t *const p_st_srr_free_space, PyObject *const p_sp_2021)
{
    bool result = true;

    if (p_sp_2021 != NULL)
    {
        char index[3];
        for (int i = 0; i < SRR_FREESPACE_OBJECT_LIST_SIZE; i++)
        {
            sprintf(index, "%d", i);
            if (i == 0)
            {
                /* boundaryType */
                result &= construct_from_u8(&(p_st_srr_free_space[i].boundaryType),
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "freeSpace"),
                                                        "freeSpaceBoundarySubSegment"),
                                                    index),
                                                "boundaryType"),
                                            add_data_in_object(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(
                                                                       PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                       "freeSpace"),
                                                                   "timestamp"),
                                                               FLOAT64, -1 * f64_time_offset_g));
            }
            else
            {
                /* boundaryType */
                result &= construct_from_u8(&(p_st_srr_free_space[i].boundaryType),
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "freeSpace"),
                                                        "freeSpaceBoundarySubSegment"),
                                                    index),
                                                "boundaryType"),
                                            PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                      PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                      "freeSpace"),
                                                                  "timestamp"));
            }
            /* existanceProbability */
            result &= construct_from_u8(&(p_st_srr_free_space[i].existanceProbability),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                        "freeSpace"),
                                                    "freeSpaceBoundarySubSegment"),
                                                index),
                                            "existanceProbability"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                  "freeSpace"),
                                                              "timestamp"));
            /* movingStatus */
            result &= construct_from_u8(&(p_st_srr_free_space[i].movingStatus),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                        "freeSpace"),
                                                    "freeSpaceBoundarySubSegment"),
                                                index),
                                            "movingStatus"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                  "freeSpace"),
                                                              "timestamp"));
            /* range */
            result &= construct_from_f32(&(p_st_srr_free_space[i].range),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                         "freeSpace"),
                                                     "freeSpaceBoundarySubSegment"),
                                                 index),
                                             "range"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                   "freeSpace"),
                                                               "timestamp"));
            /* source */
            result &= construct_from_u8(&(p_st_srr_free_space[i].source),
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                        "freeSpace"),
                                                    "freeSpaceBoundarySubSegment"),
                                                index),
                                            "source"),
                                        PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                  PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                  "freeSpace"),
                                                              "timestamp"));
            /* subSegmentID */
            result &= construct_from_u16(&(p_st_srr_free_space[i].subSegmentID),
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(
                                                     PyDict_GetItemWrapper(
                                                         PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                         "freeSpace"),
                                                     "freeSpaceBoundarySubSegment"),
                                                 index),
                                             "subSegmentID"),
                                         PyDict_GetItemWrapper(PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                   "freeSpace"),
                                                               "timestamp"));
        }
    }
    else
    {
        result = false;
    }

    return result;
}

static bool parse_srr_tap_object_signal(st_pickle_srr_tap_object_t *const p_st_srr_tap_object, PyObject *const p_sp_2021)
{
    bool result = true;
    if (p_sp_2021 != NULL)
    {
        /* criticalObjectLeftExistenceProbability */
        result &= construct_from_u16(&p_st_srr_tap_object->criticalObjectLeftExistenceProbability,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectTurnAcrossPath"),
                                             "turnAcrossPathCriticalObjectInfo"),
                                         "criticalObjectLeftExistenceProbability"),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                "criticalObjectTurnAcrossPath"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
        /* criticalObjectLeftID */
        result &= construct_from_u8(&p_st_srr_tap_object->criticalObjectLeftID,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                "criticalObjectTurnAcrossPath"),
                                            "turnAcrossPathCriticalObjectInfo"),
                                        "criticalObjectLeftID"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectTurnAcrossPath"),
                                        "timestamp"));
        /* criticalObjectLeftPOX */ // minus codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectLeftPOX,
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                    "criticalObjectTurnAcrossPath"),
                                                                "turnAcrossPathCriticalObjectInfo"),
                                                            "criticalObjectLeftPOX"),
                                                        FLOAT32, -1. * codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectLeftPOY */ // Multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectLeftPOY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectTurnAcrossPath"),
                                                        "turnAcrossPathCriticalObjectInfo"),
                                                    "criticalObjectLeftPOY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectLeftTTCX */
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectLeftTTCX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectTurnAcrossPath"),
                                             "turnAcrossPathCriticalObjectInfo"),
                                         "criticalObjectLeftTTCX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectLeftVX */
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectLeftVX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectTurnAcrossPath"),
                                             "turnAcrossPathCriticalObjectInfo"),
                                         "criticalObjectLeftVX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectLeftVY */ // multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectLeftVY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectTurnAcrossPath"),
                                                        "turnAcrossPathCriticalObjectInfo"),
                                                    "criticalObjectLeftVY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectRightExistenceProbability */
        result &= construct_from_u16(&p_st_srr_tap_object->criticalObjectRightExistenceProbability,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectTurnAcrossPath"),
                                             "turnAcrossPathCriticalObjectInfo"),
                                         "criticalObjectRightExistenceProbability"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectRightID */
        result &= construct_from_u8(&p_st_srr_tap_object->criticalObjectRightID,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                "criticalObjectTurnAcrossPath"),
                                            "turnAcrossPathCriticalObjectInfo"),
                                        "criticalObjectRightID"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectTurnAcrossPath"),
                                        "timestamp"));
        /* criticalObjectRightPOX */ // minus codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectRightPOX,
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                    "criticalObjectTurnAcrossPath"),
                                                                "turnAcrossPathCriticalObjectInfo"),
                                                            "criticalObjectRightPOX"),
                                                        FLOAT32, -1. * codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectRightPOY */ // Multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectRightPOY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectTurnAcrossPath"),
                                                        "turnAcrossPathCriticalObjectInfo"),
                                                    "criticalObjectRightPOY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectRightTTCX */
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectRightTTCX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectTurnAcrossPath"),
                                             "turnAcrossPathCriticalObjectInfo"),
                                         "criticalObjectRightTTCX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectRightVX */
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectRightVX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectTurnAcrossPath"),
                                             "turnAcrossPathCriticalObjectInfo"),
                                         "criticalObjectRightVX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* criticalObjectRightVY */ // multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_tap_object->criticalObjectRightVY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectTurnAcrossPath"),
                                                        "turnAcrossPathCriticalObjectInfo"),
                                                    "criticalObjectRightVY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectTurnAcrossPath"),
                                         "timestamp"));
        /* statusDynamicArea */
        result &= construct_from_u8(&p_st_srr_tap_object->statusDynamicArea,
                                    replace_data_in_object(PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                   "criticalObjectTurnAcrossPath"),
                                                               "statusDynamicArea"),
                                                           UINT8, 1.0, 0.0),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectTurnAcrossPath"),
                                        "timestamp"));
        /* statusSimultaneousLaneChange */
        result &= construct_from_u8(&p_st_srr_tap_object->statusTurningArea,
                                    replace_data_in_object(PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                   "criticalObjectTurnAcrossPath"),
                                                               "statusTurningArea"),
                                                           UINT8, 1.0, 0.0),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectTurnAcrossPath"),
                                        "timestamp"));
    }
    else
    {
        result = false;
    }

    return result;
}

static bool parse_srr_lcw_object_signal(st_pickle_srr_lcw_object_t *const p_st_srr_lcw_object, PyObject *const p_sp_2021)
{
    bool result = true;
    if (p_sp_2021 != NULL)
    {
        /* criticalObjectLeftExistenceProbability */
        result &= construct_from_u16(&p_st_srr_lcw_object->criticalObjectLeftExistenceProbability,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectLaneChange"),
                                             "laneChangeCriticalObjectInfo"),
                                         "criticalObjectLeftExistenceProbability"),
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                "criticalObjectLaneChange"),
                                                            "timestamp"),
                                                        FLOAT64, -1 * f64_time_offset_g));
        /* criticalObjectLeftID */
        result &= construct_from_u8(&p_st_srr_lcw_object->criticalObjectLeftID,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                "criticalObjectLaneChange"),
                                            "laneChangeCriticalObjectInfo"),
                                        "criticalObjectLeftID"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectLaneChange"),
                                        "timestamp"));
        /* criticalObjectLeftLaneChangeProbability */
        result &= construct_from_u16(&p_st_srr_lcw_object->criticalObjectLeftLaneChangeProbability,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectLaneChange"),
                                             "laneChangeCriticalObjectInfo"),
                                         "criticalObjectLeftLaneChangeProbability"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectLeftPOX */ // minus codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectLeftPOX,
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                    "criticalObjectLaneChange"),
                                                                "laneChangeCriticalObjectInfo"),
                                                            "criticalObjectLeftPOX"),
                                                        FLOAT32, -1. * codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectLeftPOY */ // Multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectLeftPOY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectLaneChange"),
                                                        "laneChangeCriticalObjectInfo"),
                                                    "criticalObjectLeftPOY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectLeftTTCX */
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectLeftTTCX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectLaneChange"),
                                             "laneChangeCriticalObjectInfo"),
                                         "criticalObjectLeftTTCX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectLeftVX */
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectLeftVX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectLaneChange"),
                                             "laneChangeCriticalObjectInfo"),
                                         "criticalObjectLeftVX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectLeftVY */ // multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectLeftVY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectLaneChange"),
                                                        "laneChangeCriticalObjectInfo"),
                                                    "criticalObjectLeftVY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectRightExistenceProbability */
        result &= construct_from_u16(&p_st_srr_lcw_object->criticalObjectRightExistenceProbability,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectLaneChange"),
                                             "laneChangeCriticalObjectInfo"),
                                         "criticalObjectRightExistenceProbability"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectRightID */
        result &= construct_from_u8(&p_st_srr_lcw_object->criticalObjectRightID,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                "criticalObjectLaneChange"),
                                            "laneChangeCriticalObjectInfo"),
                                        "criticalObjectRightID"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectLaneChange"),
                                        "timestamp"));
        /* criticalObjectRightLaneChangeProbability */
        result &= construct_from_u16(&p_st_srr_lcw_object->criticalObjectRightLaneChangeProbability,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectLaneChange"),
                                             "laneChangeCriticalObjectInfo"),
                                         "criticalObjectRightLaneChangeProbability"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectRightPOX */ // minus codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectRightPOX,
                                     add_data_in_object(PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(
                                                                PyDict_GetItemWrapper(
                                                                    PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                    "criticalObjectLaneChange"),
                                                                "laneChangeCriticalObjectInfo"),
                                                            "criticalObjectRightPOX"),
                                                        FLOAT32, -1. * codingAllInput.vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectRightPOY */ // Multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectRightPOY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectLaneChange"),
                                                        "laneChangeCriticalObjectInfo"),
                                                    "criticalObjectRightPOY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectRightTTCX */
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectRightTTCX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectLaneChange"),
                                             "laneChangeCriticalObjectInfo"),
                                         "criticalObjectRightTTCX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectRightVX */
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectRightVX,
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(
                                                 PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                 "criticalObjectLaneChange"),
                                             "laneChangeCriticalObjectInfo"),
                                         "criticalObjectRightVX"),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* criticalObjectRightVY */ // multiplied by NEGATIVE_CONV
        result &= construct_from_f32(&p_st_srr_lcw_object->criticalObjectRightVY,
                                     scale_data(PyDict_GetItemWrapper(
                                                    PyDict_GetItemWrapper(
                                                        PyDict_GetItemWrapper(
                                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                            "criticalObjectLaneChange"),
                                                        "laneChangeCriticalObjectInfo"),
                                                    "criticalObjectRightVY"),
                                                FLOAT32, NEGATIVE_CONV),
                                     PyDict_GetItemWrapper(
                                         PyDict_GetItemWrapper(
                                             PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                             "criticalObjectLaneChange"),
                                         "timestamp"));
        /* statusBlindSpotArea */
        result &= construct_from_u8(&p_st_srr_lcw_object->StatusBlindSpotArea,
                                    replace_data_in_object(PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                   "criticalObjectLaneChange"),
                                                               "statusBlindSpotArea"),
                                                           UINT8, 1.0, 0.0),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectLaneChange"),
                                        "timestamp"));
        /* statusDynamicArea */
        result &= construct_from_u8(&p_st_srr_lcw_object->StatusDynamicArea,
                                    replace_data_in_object(PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                   "criticalObjectLaneChange"),
                                                               "statusDynamicArea"),
                                                           UINT8, 1.0, 0.0),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectLaneChange"),
                                        "timestamp"));
        /* statusSimultaneousLaneChange */
        result &= construct_from_u8(&p_st_srr_lcw_object->StatusSimultaneousLaneChange,
                                    replace_data_in_object(PyDict_GetItemWrapper(
                                                               PyDict_GetItemWrapper(
                                                                   PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                                                   "criticalObjectLaneChange"),
                                                               "statusSimultaneousLaneChange"),
                                                           UINT8, 1.0, 0.0),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidanceCriticalObjects"),
                                            "criticalObjectLaneChange"),
                                        "timestamp"));
    }
    else
    {
        result = false;
    }

    return result;
}

static bool fill_lca_qualifiers_pointers(st_pickle_lca_qualifier_t *const p_st_lca_qualifier, PyObject *const p_sp_2021)
{
    bool result = true;
    if (p_sp_2021 != NULL)
    {
        /* extendedQualifierLaneChangeWarning.left */
        result &= construct_from_u8(&p_st_lca_qualifier->extendedQualifierLaneChangeWarning.left,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidance"),
                                                "lateralCollisionAvoidanceQualifiers"),
                                            "statusLaneChangeWarning"),
                                        "statusLaneChangeWarningLeft"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidance"),
                                            "lateralCollisionAvoidanceQualifiers"),
                                        "timestamp"));
        /* extendedQualifierLaneChangeWarning.right */
        result &= construct_from_u8(&p_st_lca_qualifier->extendedQualifierLaneChangeWarning.right,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(
                                                PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidance"),
                                                "lateralCollisionAvoidanceQualifiers"),
                                            "statusLaneChangeWarning"),
                                        "statusLaneChangeWarningRight"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidance"),
                                            "lateralCollisionAvoidanceQualifiers"),
                                        "timestamp"));
        /* extendedQualifierSideCollisionWarning */
        result &= construct_from_u8(&p_st_lca_qualifier->extendedQualifierSideCollisionWarning,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidance"),
                                            "lateralCollisionAvoidanceQualifiers"),
                                        "extendedQualifierSideCollisionWarning"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidance"),
                                            "lateralCollisionAvoidanceQualifiers"),
                                        "timestamp"));
        /* extendedQualifierLaneDepartureWarning */
        result &= construct_from_u8(&p_st_lca_qualifier->extendedQualifierLaneDepartureWarning,
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidance"),
                                            "lateralCollisionAvoidanceQualifiers"),
                                        "extendedQualifierLaneDepartureWarning"),
                                    PyDict_GetItemWrapper(
                                        PyDict_GetItemWrapper(
                                            PyDict_GetItemWrapper(p_sp_2021, "LateralCollisionAvoidance"),
                                            "lateralCollisionAvoidanceQualifiers"),
                                        "timestamp"));
    }
    else
    {
        result = false;
    }

    return result;
}

void read_coding_values_from_pickle(Coding_T* coding_values, PyObject *const p_calibrations)
{    
    if (p_calibrations != NULL)
    {
        /// Read coding values 
        float C_CAM_MAIN_POS_X_ASIL = get_u16_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"C_CAM_MAIN_POS_X_ASIL"));
        float C_CAM_MAIN_POS_Y_ASIL = get_u16_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"C_CAM_MAIN_POS_Y_ASIL"));
        float C_CAR_FRONT_BUMPER_POS_X_ASIL = get_u16_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"C_CAR_FRONT_BUMPER_POS_X_ASIL"));
        float C_CAR_LENGTH_ASIL = get_u16_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"C_CAR_LENGTH_ASIL"));
        float C_TRACK_WIDTH_FRONT_ASIL = get_u16_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"C_TRACK_WIDTH_FRONT_ASIL"));        
        float C_WHEEL_WIDTH_ASIL = get_u16_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"C_WHEEL_WIDTH_ASIL"));
        float C_WHEEL_BASE_ASIL = get_u16_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"C_WHEEL_BASE_ASIL"));
        float C_CAM_MAIN_DIST_TO_LEFT_WHEELCASE = get_f32_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"C_CAM_MAIN_DIST_TO_LEFT_WHEELCASE"));
        float C_CAM_MAIN_DIST_TO_RIGHT_WHEELCASE = get_f32_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"C_CAM_MAIN_DIST_TO_RIGHT_WHEELCASE"));
        float CAR_DIST_MIRROR_TO_MIRROR_ASIL = get_u16_data_from_pyobject(PyDict_GetItemWrapper(p_calibrations,"CAR_DIST_MIRROR_TO_MIRROR_ASIL"));

        /// Calculate values based on requirements
        float C_VEHICLE_WIDTH = (C_TRACK_WIDTH_FRONT_ASIL /1000) + (C_WHEEL_WIDTH_ASIL / 1000);
	    float C_DIST_CAMERA_TO_FRONT_BUMPER = (C_CAR_FRONT_BUMPER_POS_X_ASIL - C_CAM_MAIN_POS_X_ASIL) / 1000;
	    float C_DIST_CAMERA_TO_FRONT_AXLE = (C_WHEEL_BASE_ASIL - C_CAM_MAIN_POS_X_ASIL) / 1000;
	    float C_DIST_FRONT_BUMPER_TO_REAR_AXLE = (C_CAR_FRONT_BUMPER_POS_X_ASIL / 1000);
	    float C_CAMERA_LAT_POS = (C_CAM_MAIN_POS_Y_ASIL / 1000) - 2;
        CAR_DIST_MIRROR_TO_MIRROR_ASIL = CAR_DIST_MIRROR_TO_MIRROR_ASIL / 1000;
	    float C_DIST_CAMERA_TO_LEFT_WHEEL_EDGE = C_CAM_MAIN_DIST_TO_LEFT_WHEELCASE;
	    float C_DIST_CAMERA_TO_RIGHT_WHEEL_EDGE = C_CAM_MAIN_DIST_TO_RIGHT_WHEELCASE;        

        if (coding_values->vehicleCoding.C_VEHICLE_LENGTH != C_CAR_LENGTH_ASIL || coding_values->vehicleCoding.C_VEHICLE_WIDTH !=  C_VEHICLE_WIDTH ||
            coding_values->vehicleCoding.C_VEHICLE_WHEELBASE != C_WHEEL_BASE_ASIL || coding_values->vehicleCoding.C_DIST_CAMERA_TO_FRONT_BUMPER !=  C_DIST_CAMERA_TO_FRONT_BUMPER ||
            coding_values->vehicleCoding.C_DIST_CAMERA_TO_FRONT_AXLE != C_DIST_CAMERA_TO_FRONT_AXLE || coding_values->vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE !=  C_DIST_FRONT_BUMPER_TO_REAR_AXLE ||
            coding_values->vehicleCoding.C_CAMERA_LAT_POS != C_CAMERA_LAT_POS || coding_values->vehicleCoding.C_DIST_CAMERA_TO_LEFT_WHEEL_EDGE !=  C_DIST_CAMERA_TO_LEFT_WHEEL_EDGE ||
            coding_values->vehicleCoding.C_DIST_CAMERA_TO_RIGHT_WHEEL_EDGE != C_DIST_CAMERA_TO_RIGHT_WHEEL_EDGE || coding_values->vehicleCoding.C_CAR_DIST_MIRROR_TO_MIRROR !=  CAR_DIST_MIRROR_TO_MIRROR_ASIL)
        {
            LOG_ERROR("Mismatch between reference coding values and DLT! Coding values will be updated with ones in DLT");
            coding_values->vehicleCoding.C_VEHICLE_LENGTH = C_CAR_LENGTH_ASIL;
            coding_values->vehicleCoding.C_VEHICLE_WIDTH =  C_VEHICLE_WIDTH;
            coding_values->vehicleCoding.C_VEHICLE_WHEELBASE = C_WHEEL_BASE_ASIL;
            coding_values->vehicleCoding.C_DIST_CAMERA_TO_FRONT_BUMPER =  C_DIST_CAMERA_TO_FRONT_BUMPER;
            coding_values->vehicleCoding.C_DIST_CAMERA_TO_FRONT_AXLE = C_DIST_CAMERA_TO_FRONT_AXLE; 
            coding_values->vehicleCoding.C_DIST_FRONT_BUMPER_TO_REAR_AXLE =  C_DIST_FRONT_BUMPER_TO_REAR_AXLE;
            coding_values->vehicleCoding.C_CAMERA_LAT_POS = C_CAMERA_LAT_POS;
            coding_values->vehicleCoding.C_DIST_CAMERA_TO_LEFT_WHEEL_EDGE =  C_DIST_CAMERA_TO_LEFT_WHEEL_EDGE;
            coding_values->vehicleCoding.C_DIST_CAMERA_TO_RIGHT_WHEEL_EDGE = C_DIST_CAMERA_TO_RIGHT_WHEEL_EDGE;
            coding_values->vehicleCoding.C_CAR_DIST_MIRROR_TO_MIRROR =  CAR_DIST_MIRROR_TO_MIRROR_ASIL;   
        }
    }
    else
    {
        LOG_WARN("DLT missing in logs");
    }

}

/********* External Function Definition *******/

bool lca_sil_pickle_parser_load(LCA_pickle_data_t *const lca_pickle_data, PyObject *const p_pickle_data, Coding_T* coding_values)
{
    bool b_success = true;

    if (p_pickle_data == NULL)
    {
        b_success = false;
    }
    else if (lca_sil_python_is_initialized() == false)
    {
        /* error */
        LOG_ERROR("Python not initialized, aborting");
        b_success = false;
    }
    else
    {
        /// read coding values from pickle if existing
        read_coding_values_from_pickle(coding_values, PyDict_GetItemWrapper(
                                                       PyDict_GetItemWrapper(p_pickle_data,
                                                        "DLT"),
                                                        "Car Mechanical Coding Data BMW"));

        /// this value must be available and not possible to run resim without it
        PyObject *p_spi = PyDict_GetItemWrapper(p_pickle_data, "SPI");
        if (p_spi == NULL)
        {
            /* error */
            LOG_ERROR("SPI is missing in pickle file, not possible to run resimulation");
            return false;
        }

        /// this value must be available and not possible to run resim without it
        PyObject *p_sp_2021 = PyDict_GetItemWrapper(p_pickle_data, "SP2021");
        if (p_sp_2021 == NULL)
        {
            /* error */
            LOG_ERROR("SPI2021 is missing in pickle file, not possible to run resimulation");
            return false;
        }

        /// this value must be available and not possible to run resim without it
        PyObject *p_spi_eyeq = PyDict_GetItemWrapper(p_spi, "EYEQ_TO_HOST");
        if (p_spi_eyeq == NULL)
        {
            /* error */
            LOG_ERROR("EYEQ_TO_HOST is missing in pickle file, not possible to run resimulation");
            return false;
        }        
        /// Fill global time offset before parsing any other signal
        double t0spi = get_timestamp(PyDict_GetItemWrapper(PyDict_GetItemWrapper(p_spi_eyeq, "Core_Lanes_Host_protocol"), "timestamp"));
        if (t0spi == -1)
	  {
            LOG_ERROR("Core_Lanes_Host_protocol is missing in pickle file, not possible to run resimulation");
            return b_success;
	  }
        
        // 20211030A: first try to get timestamp from SP2021.VelocityAndVehicleConditionProvision.velocityVehicle.timestamp (for log of SOP_1-3)
	double t0sp2021 ;
	if ( e_pickle_ASTEP == A450_SOP4 )
	  {
            t0sp2021 = get_timestamp(PyDict_GetItemWrapper(PyDict_GetItemWrapper(PyDict_GetItemWrapper(p_sp_2021, "EgoMotionSignalProvision"), "egoMotionSignal"), "timestamp")); // 20211030A 
          }
	else
	  {
	    t0sp2021 = get_timestamp(PyDict_GetItemWrapper(PyDict_GetItemWrapper(PyDict_GetItemWrapper(p_sp_2021, "VelocityAndVehicleConditionProvision"), "velocityVehicle"), "timestamp"));	  
	  }
	
	if (t0sp2021 == -1)
	  {            
	    LOG_ERROR("VelocityAndVehicleConditionProvision is missing in pickle file, not possible to run resimulation");
                return false;
	  }
	
        if ((t0spi > (t0sp2021 + DELTATIMETHRESHOLD)) || (t0sp2021 > (t0spi + DELTATIMETHRESHOLD)))
        {
            char msg[200];
            sprintf(msg," Timestamp Error! t0spi = %f, t0sp2021 = %f and delta time stamp = %d \n", t0spi, t0sp2021, abs(t0spi - t0sp2021));
            LOG_ERROR(msg); // 20211031A
        }
        /// this logic is from Import_Pickle_data_versions.m
        if ((t0sp2021 > t0spi && t0spi != 0) || (t0sp2021 == 0))
        {
            f64_time_offset_g = t0spi;
        }
        else if ((t0sp2021 < t0spi && t0sp2021 != 0) || (t0spi == 0))
        {
            f64_time_offset_g = t0sp2021;
        }
        /// parse vision lane support to fill t0SPI as done in import_pickle_data.m; this value will be used to calculate the timeoffset for all signals
        /// this value must be available and not possible to run resim without it
        b_success &= parse_vision_lane_support_host_lanes(&lca_pickle_data->visionLaneSupport, PyDict_GetItemWrapper(p_spi_eyeq, "Core_Lanes_Host_protocol"));
        if (b_success == false)
        {
            LOG_ERROR("Core_Lanes_Host_protocol is missing in pickle file, not possible to run resimulation");
            return b_success;
        }

        /// resim can run even without this value
        kLog_Error = false;
        b_success &= parse_vision_lane_support_adj_lanes(&lca_pickle_data->visionLaneSupport, PyDict_GetItemWrapper(p_spi_eyeq, "Core_Lanes_Adjacent_protocol"));
        if (b_success == false)
        {
            LOG_WARN("Core_Lanes_Adjacent_protocol is missing in pickle file, resimulation continues");
            b_success = true;
        }

        /// resim can run even without this value
        b_success &= parse_vision_lane_support_adv(&lca_pickle_data->visionLaneSupportADV, PyDict_GetItemWrapper(p_spi_eyeq, "Core_Lanes_Applications_protocol"));
        if (b_success == false)
        {
            LOG_WARN("Core_Lanes_Applications_protocol is missing in pickle file, resimulation continues");
            b_success = true;
        }

        /// resim can run even without this value
        b_success &= parse_vision_lane_support_road_edge(&lca_pickle_data->visionLaneSupportRoadEdge, PyDict_GetItemWrapper(p_spi_eyeq, "Core_Lanes_Road_Edge_protocol"));
        if (b_success == false)
        {
            LOG_WARN("Core_Lanes_Road_Edge_protocol is missing in pickle file, resimulation continues");
            b_success = true;
        }

        /// resim can run even without this value
        b_success &= parse_vision_lane_support_semantic_lane(&lca_pickle_data->visionLaneSupportSemanticLane, PyDict_GetItemWrapper(p_spi_eyeq, "Core_Semantic_Lanes_Description_protocol"));
        if (b_success == false)
        {
            LOG_WARN("Core_Semantic_Lanes_Description_protocol is missing in pickle file, resimulation continues");
            b_success = true;
        }

        /// resim can run even without this value
        b_success &= parse_vision_free_space(lca_pickle_data->visionFreeSpace, PyDict_GetItemWrapper(p_spi_eyeq, "Core_Free_Space_EXT_protocol"));
        if (b_success == false)
        {
            LOG_WARN("Core_Free_Space_EXT_protocol is missing in pickle file, resimulation continues");
            b_success = true;
        }

        /// resim can run even without this value
        b_success &= parse_vision_object_detection(lca_pickle_data->VisionObjectDetection, PyDict_GetItemWrapper(p_spi_eyeq, "Core_Objects_protocol"));
        if (b_success == false)
        {
            LOG_WARN("Core_Objects_protocol is missing in pickle file, resimulation continues");
            b_success = true;
        }

        /// resim can run even without this value
        b_success &= parse_vision_fail_safe(&lca_pickle_data->visionFailSafe, PyDict_GetItemWrapper(p_spi_eyeq, "Core_Failsafe_protocol"));
        if (b_success == false)
        {
            LOG_WARN("Core_Failsafe_protocol is missing in pickle file, resimulation continues");
            b_success = true;
        }


        // /// resim can run even without this value
        // b_success &= parse_vision_lane_CCM(lca_pickle_data->visionLaneCCM, PyDict_GetItemWrapper(p_spi_eyeq, "Core_Lanes_CCM_protocol"));
        // if (b_success == false)
        // {
        //     LOG_WARN("Core_Lanes_CCM_protocol is missing in pickle file, resimulation continues");
        //     b_success = true;
        // }

        /// resim can run even without this value
        b_success &= parse_partner_function_output(&lca_pickle_data->LCAPartnerFunctionOutput, p_sp_2021);
        if (b_success == false)
        {
            LOG_WARN("LCAPartnerFunctionOutput is missing in pickle file, resimulation continues");
            b_success = true;
        }
        kLog_Error = true;

        /// this value must be available and not possible to run resim without it
        b_success &= parse_ego_motion(&lca_pickle_data->egoMotion, p_sp_2021);
        if (b_success == false)
        {
            LOG_ERROR("EgoMotion is missing in pickle file, not possible to run resimulation");
            return b_success;
        }

        /// this value must be available and not possible to run resim without it
        b_success &= parse_bcm_signal(&lca_pickle_data->BCMSignal, p_sp_2021);
        if (b_success == false)
        {
            LOG_ERROR("BCMSignal is missing in pickle file, not possible to run resimulation");
            return b_success;
        }

        /// this value must be available and not possible to run resim without it
        b_success &= parse_srr_scw_object_signal(&lca_pickle_data->SRRSCWObject, p_sp_2021);
        if (b_success == false)
        {
            LOG_ERROR("SRRSCWObject is missing in pickle file, not possible to run resimulation");
            return b_success;
        }

        /// this value must be available and not possible to run resim without it
        b_success &= parse_srr_lcw_object_signal(&lca_pickle_data->SRRLCWObject, p_sp_2021);
        if (b_success == false)
        {
            LOG_ERROR("SRRLCWObject is missing in pickle file, not possible to run resimulation");
            return b_success;
        }

        kLog_Error = false;
        /// resim can run even without this value
        b_success &= parse_srr_tap_object_signal(&lca_pickle_data->SRRTAPObject, p_sp_2021);
        if (b_success == false)
        {
            LOG_WARN("SRRTAPObject is missing in pickle file, resimulation continues");
            b_success = true;
        }

        /// resim can run even without this value
        b_success &= parse_srr_free_space_signal(lca_pickle_data->SRRFreeSpace, p_sp_2021);
        if (b_success == false)
        {
            LOG_WARN("SRRFreeSpace is missing in pickle file, resimulation continues");
            b_success = true;
        }

        /// resim can run even without this value
        b_success &= parse_frr_object_lists_signal(lca_pickle_data->FRRObjectList, p_sp_2021);
        if (b_success == false)
        {
            LOG_WARN("FRRObjectList is missing in pickle file, resimulation continues");
            b_success = true;
        }
        kLog_Error = true;

        /// this value must be available and not possible to run resim without it
        b_success &= fill_lca_qualifiers_pointers(&lca_pickle_data->lcaQualifier, p_sp_2021);
        if (b_success == false)
        {
            kLcaQualifierMissing = true;
            LOG_WARN("LcaQualifiers is missing in pickle file, resimulation continues");
            b_success = true;
        }
    }
    return b_success;
}

void lca_sil_pickle_parser_shutdown(void)
{
    if (NULL != st_lca_pickle_base_f64_g.p_data)
    {
        free(st_lca_pickle_base_f64_g.p_data);
    }

    if (vision_lane_support_data_type == INT32)
    {
        if (NULL != st_lca_pickle_base_s32_g.p_data)
        {
            free(st_lca_pickle_base_s32_g.p_data);
        }
    }
    else if (vision_lane_support_data_type == INT64)
    {
        if (NULL != st_lca_pickle_base_s64_g.p_data)
        {
            free(st_lca_pickle_base_s64_g.p_data);
        }
    }

    f64_time_offset_g = 0.0;

    /* Reset data structure for timing      */
    st_lca_pickle_base_f64_g.p_data = NULL;
    st_lca_pickle_base_f64_g.pf64_timestamp = NULL;
    st_lca_pickle_base_f64_g.f64_time_min = .0;
    st_lca_pickle_base_f64_g.f64_time_max = .0;
    st_lca_pickle_base_f64_g.u32_length = 0u;

    if (vision_lane_support_data_type == INT32)
    {
        st_lca_pickle_base_s32_g.p_data = NULL;
        st_lca_pickle_base_s32_g.pf64_timestamp = NULL;
        st_lca_pickle_base_s32_g.f64_time_min = .0;
        st_lca_pickle_base_s32_g.f64_time_max = .0;
        st_lca_pickle_base_s32_g.u32_length = 0u;
    }
    else if (vision_lane_support_data_type == INT64)
    {
        st_lca_pickle_base_s64_g.p_data = NULL;
        st_lca_pickle_base_s64_g.pf64_timestamp = NULL;
        st_lca_pickle_base_s64_g.f64_time_min = .0;
        st_lca_pickle_base_s64_g.f64_time_max = .0;
        st_lca_pickle_base_s64_g.u32_length = 0u;
    }
}
