#!/bin/bash
# $1 is cmdline path
# $2 is bootstub path
# $3 is bzImage path
# $4 is initrd path
# $5 is spi uart disable flag (0: default, 1: disable)
# $6 is spi controller (0:auto-detect based on SoC, 1:SPI1 (Penwell, Cloverview), 2:SPI2 (Tangier, Anniedale)
# $7 is output image

if [ $# -lt 7 ]; then
	echo -e ""
	echo -e "usage: $(basename $0) cmdline_path bootstub_path bzImage_path initrd_path spi_suppress_flag spi_type output_image"
	echo -e ""
	echo -e "  spi_suppress_flag:\t0 to enable output messages"
	echo -e "\t\t\t1 to suppress output messages"
	echo -e ""
	echo -e "  spi_type:\t\t0 auto-detect based on SoC"
	echo -e "\t\t\t1 for SPI1 (Penwell, Cloverview)"
	echo -e "\t\t\t2 for SPI2 (Tangier, Anniedale)"
	echo -e ""
	exit 1
fi

if [ ! -e "$1" ]; then
	echo "cmdline file not exist!"
	exit 1
fi

if [ ! -e "$2" ]; then
	echo "bootstub file not exist!"
	exit 1
fi

if [ ! -e "$3" ]; then
	echo "no kernel bzImage file!"
	exit 1
fi

if [ ! -e "$4" ]; then
	echo "no initrd file!"
	exit 1
fi

# convert a decimal number to the sequence that printf could recognize to output binary integer (not ASCII)
binstring ()
{
	h1=$(($1%256))
	h2=$((($1/256)%256))
	h3=$((($1/256/256)%256))
	h4=$((($1/256/256/256)%256))
	binstr=`printf "\x5cx%02x\x5cx%02x\x5cx%02x\x5cx%02x" $h1 $h2 $h3 $h4`
}

# add cmdline to the first part of boot image
cat $1 /dev/zero | dd of=$7 bs=4096 count=1

# append bootstub
cat $2 /dev/zero | dd of=$7 bs=4096 count=1 seek=1

# append bzImage and initrd 
cat $3 $4 | dd of=$7 bs=4096 seek=2

# fill bzImage_size and initrd_size
binstring `stat -L -c %s $3`
printf $binstr | dd of=$7 bs=1 seek=1024 conv=notrunc
binstring `stat -L -c %s $4`
printf $binstr | dd of=$7 bs=1 seek=1028 conv=notrunc
binstring "$5"
printf $binstr | dd of=$7 bs=1 seek=1032 conv=notrunc
binstring "$6"
printf $binstr | dd of=$7 bs=1 seek=1036 conv=notrunc

# done
echo 'Image stitch done'
exit 0
