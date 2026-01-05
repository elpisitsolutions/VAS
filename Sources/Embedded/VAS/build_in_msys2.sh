#!/bin/bash

# Setup the compiler
export PATH=/c/Mohan/Softwares/Compilers/Arm/gcc-vas/bin/:$PATH
export CROSS_COMPILE=arm-none-eabi-
export ARCH=arm

echo "----------------------------------------"
echo "Changing Directory to Bootloader Project"
echo "----------------------------------------"

# Build the Bootloader
cd at91bootstrap

echo "---------------------------"
echo "Cleaning Bootloader Project"
echo "---------------------------"

# Clean the artifacts
make mrproper

echo "---------------------------"
echo "Building Bootloader Project"
echo "---------------------------"

# Write the device config
make sama5d4_xplainednf_uboot_secure_defconfig

# Project Specific Changes
make menuconfig

# Build the bootloader
make

# Change Directory to application
cd ../application/irdm

echo "--------------------------"
echo "Building Project Directory"
echo "--------------------------"

# Clean build
# make TARGET=sama5d4-xplained clean
rm -rf build/

# Build the application
make TARGET=sama5d4-xplained VARIANT=ddram

cd ../../

# Create a flash directory
echo "------------------------"
echo "Creating Flash Directory"
echo "------------------------"
rm -rf Flash/
mkdir Flash 

echo "------------------------"
echo " Coping Build Artifacts "
echo "------------------------"

# Copy bootloader artifacts
# print "Copying Bootloader Artifacts"
cp -r at91bootstrap/build/binaries/* Flash/

# Copy Image Artifacts
cp -r application/irdm/build/sama5d4-xplained/ddram/* Flash/

