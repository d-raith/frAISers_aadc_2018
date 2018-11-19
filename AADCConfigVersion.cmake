set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 2)

set(PACKAGE_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

if(PACKAGE_FIND_VERSION_MAJOR EQUAL VERSION_MAJOR)
    if(PACKAGE_FIND_VERSION_MINOR EQUAL VERSION_MINOR)
        if(PACKAGE_FIND_VERSION_PATCH EQUAL VERSION_PATCH)
            set(PACKAGE_VERSION_EXACT true)
        endif(PACKAGE_FIND_VERSION_PATCH EQUAL VERSION_PATCH)
        if(PACKAGE_FIND_VERSION_PATCH LESS VERSION_PATCH)
            set(PACKAGE_VERSION_COMPATIBLE true)
        endif(PACKAGE_FIND_VERSION_PATCH LESS VERSION_PATCH)
    endif(PACKAGE_FIND_VERSION_MINOR EQUAL VERSION_MINOR)

    if(PACKAGE_FIND_VERSION_MINOR LESS VERSION_MINOR)
        set(PACKAGE_VERSION_COMPATIBLE true)
    endif(PACKAGE_FIND_VERSION_MINOR LESS VERSION_MINOR)
endif(PACKAGE_FIND_VERSION_MAJOR EQUAL VERSION_MAJOR)
