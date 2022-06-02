#!/bin/bash       
        # Creates new boot.img...
        if [  ! -f $OUT/ramdisk.img ] || [ ! -f $OUT/cmdline ]
        then
          echo "ramdisk.img AND/OR cmdline not found in [$OUT] dir"
          echo "Check if bootimg was created and if you set your env (source build/envsetup.sh && lunch)"
          exit 1
        fi
        PATH_SRC_ROOT=$(pwd)
        PATH_OUT_KERNEL_BUILD="out/target/product/kernel_build"
        PATH_OUT_MODULES_BUILD="out/target/product/kernel_modules/lib/modules/3.10.20*/kernel/drivers/external_drivers/intel_media/bld/clovertrail" 
	
	rm -f ramdisk ramdisk.gz 2>/dev/null
        rm -rf ramdisk_edit;mkdir ramdisk_edit
	cd ramdisk_edit || exit 1
        zcat $OUT/ramdisk.img | cpio -i
        rm lib/modules/*.ko
        find ../out/target/product/kernel_modules/lib/modules/3.10.20*/kernel/ -type f -name "*.ko" -exec cp {} lib/modules/ \;
        # Override some generated modules by vendor modules...
        find ../vendor_modules/ -type f -name "*.ko" -exec cp {} lib/modules/ \;
	for arq in $(ls lib/modules/*.ko);do strip -d $arq;done
        for arq in $(ls lib/modules/*.ko);do chmod 644 $arq;done
        # Re-sign drivers after strip
        for arq in $(ls lib/modules/*ko);do ${PATH_SRC_ROOT}/kernel/scripts/sign-file sha256 ${PATH_SRC_ROOT}/${PATH_OUT_KERNEL_BUILD}/signing_key.priv ${PATH_SRC_ROOT}/${PATH_OUT_KERNEL_BUILD}/signing_key.x509 ${arq};done
cp /home/slackware/paulo/ANDROID/FONTES/LINEAGE/device/dell/P801_NoModem/debuglog.sh ./sbin
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

