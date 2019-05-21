include(CompilerLintingOptions)

option(WARNINGS "Enable compiler warnings" ON)
if(WARNINGS)
    option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL  "Clang")
    if(WARNINGS)
        if(WARNINGS_AS_ERRORS)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
        endif()

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weverything")

        set(WARNINGS_THAT_NEED_FURTHER_REVIEWING
            cast-align
            conversion
            double-promotion
            exceptions
            exit-time-destructors
            float-conversion
            float-equal
            global-constructors
            missing-braces
            redundant-parens
            reserved-id-macro
            shorten-64-to-32
            sign-compare
            sign-conversion
            undefined-func-template
            unused-macros
        )

        set(WARNINGS_THAT_SHOULD_BE_ENABLED
            switch-enum
            unknown-pragmas
        )

        set(WARNINGS_THAT_SHOULD_BE_IGNORED # WHY it is ok to ignore

            c++98-compat                    # Code base should be modern

            c++98-compat-pedantic           # Code base should be modern

            covered-switch-default          # We want this. All values should be explicit handled AND the default case
                                            # should throw Exceptions::EnumOutOfRange

            newline-eof                     # Legacy warning, no benefit when using modern compilers and editors

            padded                          # It's not worth the effort in our domain (desktop PC with tons of memory)

            weak-vtables                    # The vtable must be duplicated in multiple translation units. Small
                                            # problem, maybe even linker will resolve this. Must add boilerplate to
                                            # fix, not worth it
        )

        foreach(WARNING ${WARNINGS_THAT_NEED_FURTHER_REVIEWING} ${WARNINGS_THAT_SHOULD_BE_ENABLED} ${WARNINGS_THAT_SHOULD_BE_IGNORED})
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-${WARNING}")
        endforeach()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
    endif()
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL  "GNU")
    if(WARNINGS)
        if(WARNINGS_AS_ERRORS)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
        endif()

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

        set(WARNINGS_THAT_NEED_FURTHER_REVIEWING
            ignored-attributes
            type-limits
        )

        set(WARNINGS_THAT_SHOULD_BE_ENABLED
            unknown-pragmas
        )

        set(WARNINGS_THAT_SHOULD_BE_IGNORED # WHY it is ok to ignore

            sign-compare                    # We are not at this level of code quality, enabling this will just result
                                            # in countless thoughtless static_casts
        )

        foreach(WARNING ${WARNINGS_THAT_NEED_FURTHER_REVIEWING} ${WARNINGS_THAT_SHOULD_BE_ENABLED} ${WARNINGS_THAT_SHOULD_BE_IGNORED})
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-${WARNING}")
        endforeach()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
    endif()
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL  "MSVC")
    if(WARNINGS)
        if(WARNINGS_AS_ERRORS)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
        endif()

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall")

        set(WARNINGS_THAT_NEED_FURTHER_REVIEWING
            4013 # undefined function
            4018 # signed/unsigned mismatch
            4047 # different levels of indirection
            4061 # switch not explicitly handling all cases
            4062 # switch not handling all cases
            4068 # unknown pragma
            4100 # unreferenced formal parameter
            4101 # unreferenced local variable
            4114 # type qualifier used more than once
            4127 # conditional expression is constant
            4131 # old-style declarator
            4133 # incompatible types
            4189 # local variable is initialized but not referenced
            4191 # unsafe conversion
            4242 # possible loss of data
            4244 # conversion, possible loss of data
            4245 # signed/unsigned mismatch
            4251 # needs to have dll-interface to be used by clients of class
            4255 # no function prototype given
            4263 # shadowing / failed override
            4264 # shadowing / failed override?
            4265 # class has virtual functions, but destructor is not virtual
            4266 # function' : no override available for virtual member function from base 'type'; function is hidden
            4267 # conversion, possible loss of data
            4275 # non dll-interface class used as base for dll-interface
            4297 # function assumed not to throw an exception but does
            4305 # truncation from double
            4309 # compile time overflow
            4312 # conversion to pointer of greater size
            4324 # structure was padded due to alignment specifier
            4339 # use of undefined type detected in CLR meta-data
            4355 # used in base member initializer list
            4365 # signed/unsigned mismatch
            4371 # layout of class may have changed from a previous version of the compiler
            4388 # signed/unsigned mismatch
            4400 # qualifier is not supported
            4435 # class1' : Object layout under /vd2 will change due to virtual base 'class2'
            4456 # shadowing
            4457 # shadowing
            4458 # shadowing
            4459 # shadowing
            4464 # relative include path contains '..'
            4472 # native enum, add an access specifier
            4477 # format string error
            4487 # shadowing / failed override
            4505 # unreferenced local function has been removed
            4514 # unreferenced inline function has been removed
            4536 # type-name exceeds meta-data limit
            4548 # expression before comma has no effect
            4571 # catch(...) semantics changed since Visual C++ 7.1
            4574 # '_SECURE_SCL' is defined to be '0': did you mean to use '#if _SECURE_SCL'?
            4582 # constructor not called implicitly
            4583 # destructor not called implicitly
            4596 # illegal qualified name in member declaration
            4619 # pragma warning number unknown
            4623 # default constructor was implicitly defined as deleted
            4625 # copy constructor was implicitly defined as deleted
            4626 # assignment operator was implicitly defined as deleted
            4633 # error in documentation comment
            4634 # error in documentation comment
            4643 # forward declaring in std namespace
            4668 # undefined preprocessor macro
            4692 # signature of non-private member contains assembly private native type
            4701 # potentially uninitialized local pointer variable used
            4702 # unreachable code
            4703 # potentially uninitialized local pointer variable used
            4710 # function not inlined
            4711 # function selected for automatic inline expansion
            4714 # function 'function' marked as __forceinline not inlined
            4715 # not all control paths return a value
            4774 # format string expected in argument 2 is not a string literal
            4800 # forcing value to bool
            4805 # unsafe mix of types
            4820 # padding added
            4838 # conversion, requires a narrowing conversion
            4868 # compiler may not enforce left-to-right evaluation order in braced initializer list
            4917 # a GUID can only be associated with a class, interface or namespace
            4946 # reinterpret_cast used between related classes
            4987 # nonstandard extension used
            4996 # deprecated declaration
            5026 # move constructor was implicitly defined as deleted
            5027 # move assignment operator was implicitly defined as deleted
            5031 # pragma warning push/pop in different files
            5032 # pragma warning push with no corresponding pop
            5038 # wrong initialization order in constructor
            5039 # pointer or reference to potentially throwing function passed to extern C function under -EHc. Undefined behavior may occur if this function throws an exception.
            5045 # compiler will insert Spectre mitigation for memory load
        )

        set(WARNINGS_THAT_SHOULD_BE_ENABLED
        )

        set(WARNINGS_THAT_SHOULD_BE_IGNORED # * Description of warning
                                            # * WHY it is ok to ignore
        )

        foreach(WARNING ${WARNINGS_THAT_NEED_FURTHER_REVIEWING} ${WARNINGS_THAT_SHOULD_BE_ENABLED} ${WARNINGS_THAT_SHOULD_BE_IGNORED})
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd${WARNING}")
        endforeach()

        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ignore:4099") #PDB '...' was not found with '...' or at '...'; linking objects as if no debug info
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w")
    endif()
else()
   message(FATAL_ERROR "Unknown compiler: ${CMAKE_CXX_COMPILER_ID}")
endif()
