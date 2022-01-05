/****************** Includes ******************/
#define PY_ARRAY_UNIQUE_SYMBOL lca_sil_pickle_python_ARRAY_API
#include <common/lca_sil_python.h>
#include <common/lca_sil_log.h>

/****************** Global Variables *******************/
/// To enable/disable printing warning messages
bool g_toggle_printing_warning_messages = true;

/****************** Defines *******************/
/********* Local Function Decleration *********/
/********* Local Variable Declearation ********/
static bool b_python_is_initialized_g = false;

static PyObject *p_pickle_module_g = NULL;

static PyObject *p_io_module_g = NULL;

/********* Local Function Definition **********/
/********* External Function Definition *******/

bool lca_sil_python_initialize(void)
{
    if (false == b_python_is_initialized_g)
    {
        p_pickle_module_g = NULL;
        p_io_module_g = NULL;

        Py_Initialize();
        import_array();
        if (Py_IsInitialized() > 0)
        {
            p_pickle_module_g = PyImport_ImportModule("pickle");
            p_io_module_g = PyImport_ImportModule("io");
        }
        else
        {
            b_python_is_initialized_g = false;
        }

        if ((p_pickle_module_g == NULL) || (p_io_module_g == NULL))
        {
            LOG_ERROR("Error in initializing Python. Will now terminate... BOOM");
        }
        else
        {
            b_python_is_initialized_g = true;
        }
    }
    else
    {
        LOG_WARN("Python init function called multiple times");
    }

    return b_python_is_initialized_g;
}

bool lca_sil_python_is_initialized(void)
{
    return b_python_is_initialized_g;
}

void lca_sil_python_shutdown(void)
{
    Py_CLEAR(p_pickle_module_g);
    Py_CLEAR(p_io_module_g);
    p_io_module_g = NULL,
    p_pickle_module_g = NULL;

    Py_Finalize();
    b_python_is_initialized_g = false;
}

PyObject *lca_sil_python_read_file(const char *const p_file)
{
    PyObject *p_fd = NULL;
    if (p_file == NULL)
    {
        LOG_ERROR("[lca_sil_python_read_file] NULL pointer input");
    }
    else
    {
        PyObject *p_handle = PyObject_CallMethod(p_io_module_g, "open", "ss", p_file, "rb");

        if (p_handle == NULL)
        {
            LOG_ERROR_2("[lca_sil_python_read_file] Python error in opening file ", p_file);
        }
        else
        {
            p_fd = PyObject_CallMethod(p_pickle_module_g, "load", "O", p_handle);

            /* Close file handle again as data is already parsed into p_fd */
            (void *)PyObject_CallMethod(p_handle, "close", NULL);
            Py_CLEAR(p_handle);
        }
    }

    return p_fd;
}

bool lca_sil_python_write_file(const char *p_file, PyObject *const p_data)
{
    bool b_success = true;

    PyObject *p_file_handler = PyObject_CallMethod(p_io_module_g, "open", "ss", p_file, "wb");

    if (p_file_handler == NULL)
    {
        LOG_ERROR_2("Unable to open file ", p_file);
        b_success = false;
    }
    else
    {
        /// Extract qualifiers from original data and fill into new dictionary
        /// with same hierarchy
        PyObject *temp_dictionary = PyDict_New();
        PyDict_SetItemString(temp_dictionary, "LateralCollisionAvoidance", PyDict_GetItemWrapper(PyDict_GetItemWrapper(p_data, "SP2021"), "LateralCollisionAvoidance"));
        PyObject *qualifiers_dictionary = PyDict_New();
        PyDict_SetItemString(qualifiers_dictionary, "SP2021", temp_dictionary);

        PyObject_CallMethod(p_pickle_module_g, "dump", "OOi", qualifiers_dictionary, p_file_handler, 4);

        PyObject_CallMethod(p_file_handler, "flush", NULL);
        PyObject_CallMethod(p_file_handler, "close", NULL);

        Py_CLEAR(p_file_handler);
        Py_CLEAR(temp_dictionary);
        Py_CLEAR(qualifiers_dictionary);
    }
    return b_success;
}

PyObject *PyDict_GetItemWrapper(PyObject *const pobj_input_object, char *const key_string)
{
    /// this function returns a borrowed reference; no need to call decrease reference
    if (pobj_input_object == NULL)
    {
        return NULL;
    }
    else
    {
        // return New Reference
        PyObject *pobj_temp_key = PyUnicode_FromString(key_string);
        // returns NULL if object is not found in dictionary, else pointer to object in dict
        PyObject *pobj_object_found = PyDict_GetItemWithError(pobj_input_object, pobj_temp_key);
        if (NULL == pobj_object_found && g_toggle_printing_warning_messages)
        {
            LOG_WARN("Following signal is missing in pickle file: ");
            LOG_WARN(key_string);
        }
        // destruct object
        Py_CLEAR(pobj_temp_key);
        // this is a borrowed reference, no need to deallocated
        return pobj_object_found;
    }
}
