#!/bin/bash

#
# File descriptor 6 will output the the original stderr of the
# invoked shell. We do this so that a function can directly exit on failure...
# but still output its failure message.
if [ -e /proc/self/fd/6 ] ; then
    echo "The 6th file descriptor is already open"
    echo "Change redirections in (readlink -f $0)"
    exit 1
fi
exec 6>&2
exec 2>&1


function exit_on_error {
    if [ "$1" -ne 0 ]; then
        exit 1
    fi
}



# defaults
TOP=`pwd`
# Default the -j factor to a bit less than the number of CPUs
if [ -n "$MAKEFLAGS" ] ; then
    # Avoid setting the number of jobs in recursive make
    _jobs=0
elif [ -e /proc/cpuinfo ] ; then
    _jobs=`grep -c processor /proc/cpuinfo`
    _jobs=$(($_jobs * 2 * 8 / 10))
elif [ -e /usr/sbin/sysctl ] ; then
    _jobs=`/usr/sbin/sysctl -n hw.ncpu`
    _jobs=$(($_jobs * 2 * 8 / 10))
else
    _jobs=1
    echo "WARNING: Unavailable to determine the number of CPUs, defaulting to ${_jobs} job."
fi
_kernel_only=0
_test=0
_clean=""
_logfile_prefix=`date "+build.%Y%m%d%H%M"`
_nnn=0
_logfile=""
_preserve_kernel_config=""
_menuconfig="true"
_soc_type="mfld"
_host_os=`uname -s | tr '[:upper:]' '[:lower:]'`
TARGET_BOARD_PLATFORM="clovertrail"
KERNEL_SRC_DIR=$TOP/kernel/

init_variables() {

    # Ensure no extra signs are used on generated version
    export LOCALVERSION=""

    if [ -z "${TARGET_TOOLS_PREFIX}" ]; then
        echo >&6 "Warning: TARGET_TOOLS_PREFIX was not set."
        TARGET_TOOLS_PREFIX="$TOP/prebuilts/gcc/${_host_os}-x86/x86/i686-linux-android-4.7/bin/i686-linux-android-"

    fi
    if [ -z "${CCACHE_TOOLS_PREFIX}" ]; then
        echo >&6 "Warning: CCACHE_TOOLS_PREFIX was not set."
        CCACHE_TOOLS_DIR=$TOP/prebuilts/misc/${_host_os}-x86/ccache
    fi
    export PATH="$ANDROID_BUILD_PATHS:$PATH"

    # force using minigzip instead of gzip to build bzimage
    #export PATH="$TOP/vendor/intel/support:$PATH"
	export PATH="$TOP/prebuilts/gcc/${_host_os}-x86/x86/i686-linux-android-4.7/bin:$PATH"
	export CROSS_COMPILE="`basename ${TARGET_TOOLS_PREFIX}`"
	
    if [ -z "$kernel_build_64bit" -a -z "$CROSS_COMPILE" ];then
        export CROSS_COMPILE="`basename ${TARGET_TOOLS_PREFIX}`"
        if [ ! -z ${USE_CCACHE} ]; then
           export PATH="${CCACHE_TOOLS_DIR}:$PATH"
           export CROSS_COMPILE="ccache $CROSS_COMPILE"
        fi
    fi

    if [ -z "$kernel_build_64bit" ]; then
        export ARCH=i386
	KERNEL_BUILD_FLAGS="ANDROID_TOOLCHAIN_FLAGS=-mno-android"
    else
        export ARCH=x86_64
	KERNEL_BUILD_FLAGS="ANDROID_TOOLCHAIN_FLAGS="
    fi

    echo >&6 "ARCH: $ARCH"
    echo >&6 "CROSS_COMPILE: $CROSS_COMPILE"
    echo >&6 "PATH: $PATH"

    if [ -z "${TARGET_BOARD_PLATFORM}" ]; then
        echo "No custom board specified"
        exit_on_error 2
    fi
    VENDOR=intel

    case "${TARGET_BOARD_PLATFORM}" in
    medfield)
       _soc_type="mfld"
        ;;
    clovertrail)
        _soc_type="ctp"
        ;;
    merrifield)
        _soc_type="mrfl"
        ;;
    baytrail)
        _soc_type="byt"
        ;;
    moorefield)
        _soc_type="moor"
        ;;
    *)
        echo "Unknown platform specified \"${TARGET_BOARD_PLATFORM}\""
        exit_on_error 2
        ;;
    esac

    [ x"$PRODUCT_OUT" == x"" ] && PRODUCT_OUT=${TOP}/out/target/product/${TARGET_DEVICE}
    KERNEL_FILE=${PRODUCT_OUT}/kernel
    KERNEL_BUILD_DIR=${PRODUCT_OUT}/kernel_build

}

make_kernel() {
    local KMAKEFLAGS=("ARCH=${ARCH}" "O=${KERNEL_BUILD_DIR}" "${KERNEL_BUILD_FLAGS}")
    local njobs=""
    if [ "${_jobs}" != 0 ] ; then
      njobs="-j${_jobs}"
    fi

    mkdir -p ${KERNEL_BUILD_DIR}

    cd $KERNEL_SRC_DIR

    if [ -z "$_preserve_kernel_config" ]; then
        rm -f ${KERNEL_BUILD_DIR}/.config
    fi
    if [ "$_clean" ]; then
        make "${KMAKEFLAGS[@]}" mrproper
    fi
set -x
    if [ ! -e ${KERNEL_BUILD_DIR}/.config ]; then
        echo "making kernel ${KERNEL_BUILD_DIR}/.config file"
        cp arch/x86/configs/${ARCH}_${_soc_type}_defconfig ${KERNEL_BUILD_DIR}/.config
        make V=1 "${KMAKEFLAGS[@]}" defoldconfig
        exit_on_error $? quiet
    fi
    if "$_menuconfig" ; then
        cp ${KERNEL_BUILD_DIR}/.config ${KERNEL_BUILD_DIR}/.config.saved
        make "${KMAKEFLAGS[@]}" menuconfig
        diff -up  ${KERNEL_BUILD_DIR}/.config.saved ${KERNEL_BUILD_DIR}/.config |grep CONFIG_ |grep -v '@@'| grep + |sed 's/^+//' >>user_diffconfig
        rm ${KERNEL_BUILD_DIR}/.config.saved
        echo =========
        echo
        echo `pwd`/user_diffconfig modified accordingly. You can save your modifications to the appropriate config file
        echo
        echo =========
    fi

    make "${KMAKEFLAGS[@]}" ${njobs} #bzImage

    exit_on_error $? quiet
set +x

    mkdir -p `dirname ${KERNEL_FILE}`
    cp ${KERNEL_BUILD_DIR}/arch/x86/boot/bzImage ${KERNEL_FILE}
    exit_on_error $? quiet

    make_modules
    exit_on_error $? quiet

    cd ${TOP}
}

make_modules() {
    local MODULE_SRC=${PRODUCT_OUT}/kernel_modules
    local MODULE_DEST=${PRODUCT_OUT}/root/lib/modules

    echo "  Making driver modules..."

    rm -fr ${MODULE_SRC}
    rm -fr ${MODULE_DEST}

    if [ ! -d ${MODULE_SRC} ]; then
        mkdir -p ${MODULE_SRC}
    fi
    if [ ! -d ${MODULE_DEST} ]; then
        mkdir -p ${MODULE_DEST}
    fi

    make "${KMAKEFLAGS[@]}" ${njobs} modules
    exit_on_error $? quiet

    make "${KMAKEFLAGS[@]}" ${njobs} modules_install \
        INSTALL_MOD_PATH=${MODULE_SRC}
    exit_on_error $? quiet

    find ${MODULE_SRC} -name *.ko -exec cp -vf {} ${MODULE_DEST} \;
    exit_on_error $? quiet
}


# Build a kernel module from source that is not in the kernel build directory
make_module_external() {

    cd $KERNEL_SRC_DIR

    if [ ! -f ${KERNEL_FILE} ]; then
        echo >&6 "The kernel must be built first. File not found: ${KERNEL_FILE}"
        exit 1
    fi

    make_module_external_fcn
    exit_on_error $? quiet

    cd ${TOP}
}

run_depmod() {

    local MODULE_SRC=${PRODUCT_OUT}/kernel_modules/
    local MODULE_DEST=${PRODUCT_OUT}/root/lib/modules
    local VERSION_TAG=$(echo $MODULE_SRC/lib/modules/*/ | awk -F'/' '{print $(NF-1)}')

    tmp_dir=$(mktemp -d ${TMPDIR:-/tmp}/depmod.XXXXXX)
    tmp_mod_dir=${tmp_dir}/lib/modules/$VERSION_TAG

    mkdir -p "$tmp_mod_dir"

    find ${MODULE_DEST} -name *.ko -exec cp -f {} ${tmp_mod_dir} \;
    exit_on_error $? quiet

    /sbin/depmod -b $tmp_dir $VERSION_TAG

    find ${tmp_mod_dir} -name modules.* -exec cp -f {} ${MODULE_DEST} \;
    exit_on_error $? quiet

   rm -fr "$tmp_dir"
}

# Build a kernel module from source that is not in the kernel build directory
#   Launch the build directly from the module directory
make_module_external_in_directory() {
    local njobs=""
    if [ "${_jobs}" != 0 ] ; then
      njobs="-j${_jobs}"
    fi

    if [ ! -d ${EXTERNAL_MODULE_IN_DIRECTORY} ]; then
        echo >&6 "Module path not found: ${EXTERNAL_MODULE_IN_DIRECTORY}"
        exit 1
    fi

    cd $EXTERNAL_MODULE_IN_DIRECTORY

    if [ ! -f ${KERNEL_FILE} ]; then
        echo >&6 "The kernel must be built first. File not found: ${KERNEL_FILE}"
        exit 1
    fi

    local MODULE_DEST_TMP=${PRODUCT_OUT}/$(basename $EXTERNAL_MODULE_IN_DIRECTORY)
    local MODULE_DEST=${PRODUCT_OUT}/root/lib/modules
 
    make ARCH=${ARCH} KLIB=${MODULE_DEST_TMP} KLIB_BUILD=${KERNEL_BUILD_DIR} \
        ${njobs} ${KERNEL_BUILD_FLAGS} ${EXTRA_MAKEFLAGS}
    exit_on_error $? quiet

    rm -rf ${MODULE_DEST_TMP}
    mkdir -p ${MODULE_DEST_TMP}

    make ARCH=${ARCH} INSTALL_MOD_STRIP=${STRIP_MODE} KLIB=${MODULE_DEST_TMP} \
        KLIB_BUILD=${KERNEL_BUILD_DIR} ${njobs} ${KERNEL_BUILD_FLAGS} \
        ${EXTRA_MAKEFLAGS} install-modules
    exit_on_error $? quiet

    find ${MODULE_DEST_TMP} -name *.ko -exec cp -vf {} ${MODULE_DEST} \;
    exit_on_error $? quiet

    cd ${TOP}
}

make_module_external_fcn() {
    local MODULE_SRC=${PRODUCT_OUT}/kernel_modules
    local MODULE_DEST=${PRODUCT_OUT}/root/lib/modules
    local KMAKEFLAGS=("ARCH=${ARCH}" "O=${KERNEL_BUILD_DIR}" "${KERNEL_BUILD_FLAGS}")
    local modules_name=""
    local modules_file=""
    local njobs=""
    if [ "${_jobs}" != 0 ] ; then
      njobs="-j${_jobs}"
    fi
    echo "  Making driver modules from external source directory..."

    make "${KMAKEFLAGS[@]}" ${njobs} M=${TOP}/${EXTERNAL_MODULE_DIRECTORY} \
        ${EXTRA_MAKEFLAGS}
    exit_on_error $? quiet

    modules_file=${TOP}/${EXTERNAL_MODULE_DIRECTORY}/`basename ${EXTERNAL_MODULE_DIRECTORY}`.list

    make "${KMAKEFLAGS[@]}" ${njobs} M=${TOP}/${EXTERNAL_MODULE_DIRECTORY} \
        ${EXTRA_MAKEFLAGS} modules_install \
        INSTALL_MOD_PATH=${MODULE_SRC} \
        | tee $modules_file
    exit_on_error $? quiet

    modules_name=`cat $modules_file | grep -o -e "[a-zA-Z0-9_\.\-]*.ko"`
    rm -f $modules_file

    for module in $modules_name
    do
        find ${MODULE_SRC} -name ${module} -exec cp -vf {} ${MODULE_DEST} \;
        exit_on_error $? quiet
    done
}



usage() {
    echo "Usage: $0 <options>..."
    echo ""
    echo " -j [jobs]                # of jobs to run simultaneously.  0=automatic"
    echo " -k                       build kernel only"
    echo " -t                       testtool build"
    echo " -v                       verbose (V=1) build"
    echo " -C                       clean first"
    echo " -M                       external module source directory"
    echo " -X                       external module source directory - in module build"
    echo " -B                       Build a 64bit kernel"
    echo " -f                       Extra flags to pass to make"
}

main() {
    cd ${TOP}/kernel
	make clean
    make mrproper
    cd ../
    echo "Make mrproper finished."
    while getopts vBM:j:kthCo:X:f: opt
    do
        case "${opt}" in
        v)
            VERBOSE="V=1"
            ;;
        B)
            kernel_build_64bit=1
            ;;
        h)
            usage
            exit 0
            ;;
        M)
            EXTERNAL_MODULE_DIRECTORY="${OPTARG}"
            ;;
        X)
            EXTERNAL_MODULE_IN_DIRECTORY="${OPTARG}"
            ;;
        j)
            if [ ${OPTARG} -gt 0 ]; then
                _jobs=${OPTARG}
            else
                # Default the -j factor to a bit less than the number of CPUs
                if [ -e /proc/cpuinfo ] ; then
                    _jobs=`grep -c processor /proc/cpuinfo`
                    _jobs=$(($_jobs * 2 * 8 / 10))
                elif [ -e /usr/sbin/sysctl ] ; then
                    _jobs=`/usr/sbin/sysctl -n hw.ncpu`
                    _jobs=$(($_jobs * 2 * 8 / 10))
                else
                    _jobs=1
                    echo "WARNING: Unavailable to determine the number of CPUs, defaulting to ${_jobs} job."
                fi
            fi
            ;;
        k)
            _kernel_only=1
            echo >&6 "Kernel will be built but will not be placed in a boot image."
            ;;
        t)
            export TARGET_BUILD_VARIANT=tests
            _test=1
            ;;
        C)
            _clean=1
            ;;
        o)
            if [ "x${OPTARG}" == "xmenuconfig" ]
            then
                    _menuconfig=true
            fi
            ;;
        f)
            EXTRA_MAKEFLAGS=${OPTARG}
            ;;
        ?)
            echo "Unknown option"
            usage
            exit 0
            ;;
        esac
    done

    init_variables "$TARGET_DEVICE"

    if [ "$EXTERNAL_MODULE_DIRECTORY" ]; then
        echo >&6
        echo >&6 "Building external module for $TARGET_DEVICE"
        echo >&6 "------------------------------------------------"
        make_module_external ${TARGET_DEVICE}
    elif [ "$EXTERNAL_MODULE_IN_DIRECTORY" ]; then
        echo >&6
        echo >&6 "Building external module for $TARGET_DEVICE"
        echo >&6 "  from $EXTERNAL_MODULE_IN_DIRECTORY"
        echo >&6 "------------------------------------------------"
        make_module_external_in_directory ${TARGET_DEVICE}
    else
        echo >&6
        echo >&6 "Building kernel for $TARGET_DEVICE"
        echo >&6 "---------------------------------"
        make_kernel ${TARGET_DEVICE}
        exit_on_error $?

	

#		cp ramdisk.img out/target/product/ramdisk.img
#		cp boot_cmdline out/target/product/boot_cmdline
#		cp bootstub out/target/product/bootstub
#		stitch.sh  out/target/product/boot_cmdline out/target/product/bootstub out/target/product/kernel out/target/product/ramdisk.img 0 0 out/target/product/boot.unsigned
#        ./gen_os --input out/target/product/boot.unsigned --output out/target/product/boot.img --xml MOS_OTA.XML --platform-type MFDC0 --sign-with isu

        # Creates new boot.img...
        if [  ! -f $OUT/ramdisk.img ] || [ ! -f $OUT/cmdline ]
        then
          echo "ramdisk.img AND/OR cmdline not found in [$OUT] dir"
          echo "Check if bootimg was created and if you set your env (source build/envsetup.sh && lunch)"
          exit 1
        fi
        PATH_SRC_ROOT=$(pwd)
        PATH_OUT_KERNEL_BUILD="out/target/product/kernel_build"
        PATH_OUT_MODULES_BUILD="out/target/product/kernel_modules/lib/modules/3.10.20/kernel/drivers/external_drivers/intel_media/bld/clovertrail" 
	
	rm -f ramdisk ramdisk.gz 2>/dev/null
        rm -rf ramdisk_edit;mkdir ramdisk_edit
	cd ramdisk_edit || exit 1
        zcat $OUT/ramdisk.img | cpio -i
        rm lib/modules/*.ko
        find ../out/target/product/kernel_modules/lib/modules/3.10.20*/kernel/ -type f -name "*.ko" -exec cp {} lib/modules/ \;
	# Override some generated modules by vendor modules...
	find ../vendor_modules/ -type f -name "*.ko" -exec cp {} lib/modules/ \;
cp /home/slackware/paulo/ANDROID/FONTES/LINEAGE/device/dell/P801_NoModem/debuglog.sh ./sbin
	for arq in $(ls lib/modules/*.ko);do strip -d $arq;done
	for arq in $(ls lib/modules/*.ko);do chmod 644 $arq;done
        # Re-sign drivers after strip
        for arq in $(ls lib/modules/*ko);do ${PATH_SRC_ROOT}/kernel/scripts/sign-file sha256 ${PATH_SRC_ROOT}/${PATH_OUT_KERNEL_BUILD}/signing_key.priv ${PATH_SRC_ROOT}/${PATH_OUT_KERNEL_BUILD}/signing_key.x509 ${arq};done
        find | cpio -o --format='newc' > ../ramdisk
        cd ..
        gzip ramdisk

         # Now pack it again, using stock boot.img as base for headers & other stuff...
	./pack_intel Venue_8_WiFi_KK4.4.2v1.33.boot.img out/target/product/kernel ramdisk.gz $OUT/cmdline new_boot.img
        if [ $? -ne 0 ]
        then
          echo "pack_intel failure"
          exit 1
        fi
        echo "new_boot.img created"
        ls -l new_boot.img
        rm -rf ramdisk_edit
        rm -f ramdisk.gz
    fi
    #run_depmod
    exit 0
}

main $*
