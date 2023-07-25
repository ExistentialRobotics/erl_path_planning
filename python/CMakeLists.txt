if (BUILD_PYTHON)
    erl_generate_python_package(
            PY_PACKAGE_DIR ${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}
            PYBIND_MODULE_NAME py${PROJECT_NAME}
            PY_MODULES search_planning
            DEPENDS_PY_PACKAGES erl_common erl_geometry erl_env
    )
endif ()
