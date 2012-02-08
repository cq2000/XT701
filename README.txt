######Pre-requisties

For general Android prerequisites, see http://source.android.com/download

1. Get Google repos (except for kernel) from : http://android.git.kernel.org 
 

2. Under common folder (e.g. /home/foo) copy over the components downloaded in step 1 as well as contents of this pacakge.


3. From the common folder unpack all the archives in this package by running : 
./unpack_archive.sh


4. Building Linux Kernel and kernel modules

    make -f kernel/kernel.mk BLD_CONF=sholest_cu


5. Building Linux Boot Loader (lbl)

    make lbl_bin -f motorola/bsp/lbl/Makefile


6. Building user-space copyleft components


	a. Set env variables (for *generic* product):
   
        . build/envsetup.sh
        choosecombo

	b. Build
   
        make -j2 -k -f build/core/main.mk BOARD_HAVE_BLUETOOTH=true \
           TARGET_BOARD_PLATFORM=omap3 HARDWARE_OMX=true \
           BOARD_WLAN_TI_STA_DK_ROOT=system/wlan/ti/wilink_6_1 BOARD_WPA_SUPPLICANT_DRIVER=CUSTOM \
           BOARD_GPS_LIBRARIES= <targetname>
   
Here, `<targetname>` is the name of the component you want to build, such as `iptables` or 
`hciconfig`.

