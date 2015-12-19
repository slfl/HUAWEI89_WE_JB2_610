# Info for Java
export JAVA_HOME=/usr/lib/jvm/java-6-oracle

# Set patch for ToolChain
export CROSS_COMPILE=~/toolchains/arm-linux-androideabi-4.7.7/bin/arm-linux-androideabi-

# Workaround for + appended on kernelrelease, may not be required
#export LOCALVERSION=
export ARCH=arm
export TARGET_BUILD_VARIANT=user
export MTK_ROOT_CUSTOM=../mediatek/custom/ MTK_PATH_SOURCE=../mediatek/kernel/ MTK_PATH_PLATFORM=../mediatek/platform/mt6589/kernel/ ARCH_MTK_PLATFORM=mt6589
export TARGET_PRODUCT=huawei89_we_jb2

# This is essential to build a working kernel!
export KBUILD_BUILD_USER=Mansi
export KBUILD_BUILD_HOST=MSI

# Build command
cd kernel && make
