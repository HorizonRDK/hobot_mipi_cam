#!/bin/sh

if [ -z "${LOGLEVEL}" ]; then
export LOGLEVEL=3
fi

grep "hobot.socver=5.2.0" /proc/cmdline >/dev/null 2>&1
if [[ $? -eq 0 ]]
then
echo "J5 2.0 Board Test"
export LD_LIBRARY_PATH=/app/bin/vps/vpm/lib/v2.0:$LD_LIBRARY_PATH
else
echo "J5 1.0 Board Test"
export LD_LIBRARY_PATH=/app/bin/vps/vpm/lib/v1.0:$LD_LIBRARY_PATH
fi

COMMON_DIR=/app/bin/vps/vpm/
#OUTPUT_DIR_PREFIX=/userdata/vps/vpm/

ispcfg_idx="0 0 0 0 1 2 0 2 1 7 5 0 1 2 10 0 0 3 0"
static_param="256 512 960 0 128 2 32 32 512 2 16 8 16 512 8 0 0 512 0"
dynamic_param="0 0 0 1920 1920 0 0 0 0 0 0 0 0 0 0 16 512 0 16384"
cam_restart=0
vio_restart=0

if [ $2 ];then
	cam_rstart=$2
fi
if [ $3 ];then
	vio_rstart=$3
fi

#=================================================A For ddr feedback case===============================================

if test "$1" == "ddr_gdc_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_1080p/vpm_config.json" -G 0  -w "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_2160p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_2160p/vpm_config.json" -U "/app/bin/vps/vpm/res/2160p_0_affine.json"  -G 0  -w "/app/bin/vps/vpm/res/4k.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_1080p_rotate90"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_1080p/vpm_config.json" -G 90  -U "/app/bin/vps/vpm/res/1080p_90_affine.json" -w "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_1080p_rotate180"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_1080p/vpm_config.json" -G 180  -U "/app/bin/vps/vpm/res/1080p_180_affine.json" -w "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_1080p_rotate270"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_1080p/vpm_config.json" -G 270  -U "/app/bin/vps/vpm/res/1080p_270_affine.json" -w "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_1080p_advance"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_1080p/vpm_config.json" -l 10  -w "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_process_adv \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_1080p_advance_user"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_1080p/vpm_config.json" -l 10  -w "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_process_adv_user \

elif test "$1" == "ddr_gdc_720p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_720p/vpm_config.json" -G 0  -U "/app/bin/vps/vpm/res/720p_0_affine.json" -w "/app/bin/vps/vpm/res/720p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_720p_custom0"; then
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_720p/vpm_config.json" -G 0 -l 10 -U "/app/bin/vps/vpm/res/camera_0_layout.json" -w "/app/bin/vps/vpm/res/720p_fisheye_0.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_720p_custom1"; then
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_720p/vpm_config.json" -G 180 -l 10 -U "/app/bin/vps/vpm/res/camera_1_layout.json" -w "/app/bin/vps/vpm/res/720p_fisheye_1.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_720p_custom2"; then
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_720p/vpm_config.json" -G 90 -l 10 -U "/app/bin/vps/vpm/res/camera_2_layout.json" -w "/app/bin/vps/vpm/res/720p_fisheye_2.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc_720p_custom3"; then
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc_720p/vpm_config.json" -G 270 -l 10 -U "/app/bin/vps/vpm/res/camera_3_layout.json" -w "/app/bin/vps/vpm/res/720p_fisheye_3.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.gdc_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_gdc"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_gdc | grep ^ddr_gdc)
	do
		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc/$json" \
		-G 0 -M 1 -m 1 -U "/app/bin/vps/vpm/res/1080p_0_affine.json" \
		-w "/app/bin/vps/vpm/res/1080p.yuv" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmGdcTest.multi_gdc_feedback
	done

elif test "$1" == "ddr_gdc_dul"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_gdc | grep dul)
	do
		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc/$json" \
		-G 0 -M 3 -m 1 -U "/app/bin/vps/vpm/res/1080p_0_affine.json" \
		-w "/app/bin/vps/vpm/res/1080p.yuv" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmGdcTest.multi_gdc_feedback
	done

elif test "$1" == "ddr_gdc_octu"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_gdc | grep octu)
	do
		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc/$json" \
		-G 0 -M 255 -m 1 -U "/app/bin/vps/vpm/res/1080p_0_affine.json" \
		-w "/app/bin/vps/vpm/res/1080p.yuv" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmGdcTest.multi_gdc_feedback
	done

elif test "$1" == "ddr_gdc_oxy"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_gdc | grep oxy)
	do
		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc/$json" \
		-G 0 -M 65535 -m 1 -U "/app/bin/vps/vpm/res/1080p_0_affine.json" \
		-w "/app/bin/vps/vpm/res/1080p.yuv" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmGdcTest.multi_gdc_feedback
	done

elif test "$1" == "ddr_gdc_oxy_rotate90"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_gdc | grep oxy)
	do
		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_gdc/$json" \
		-G 90 -M 65535 -m 1 -U "/app/bin/vps/vpm/res/1080p_90_affine.json" \
		-w "/app/bin/vps/vpm/res/1080p.yuv" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmGdcTest.multi_gdc_feedback
	done

elif test "$1" == "ddr_pym_misc"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_pym | grep ^pym)
	do
		echo "$json"

		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym/$json" -m 1 -l 10 \
		-z "/app/bin/vps/vpm/res/1080p.yuv" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmPymTest.feedback_pym

		if [[ $? -ne 0 ]];then
			exit 1
		fi
	done

elif test "$1" == "ddr_pym_dul"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_pym | grep dul_pym)
	do
		echo "$json"

		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym/$json" -m 1 -l 10 -M 3 \
		-z "/app/bin/vps/vpm/res/1080p.yuv" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmPymTest.multi_pym_feedback

		if [[ $? -ne 0 ]];then
			exit 1
		fi
	done

elif test "$1" == "ddr_pym_octu"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_pym | grep octu_pym)
	do
		echo "$json"

		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym/$json" -m 1 -l 10 -M 65535 \
		-z "/app/bin/vps/vpm/res/1080p.yuv" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmPymTest.multi_pym_feedback

		if [[ $? -ne 0 ]];then
			exit 1
		fi
	done


elif test "$1" == "ddr_pym0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym0_1080p/vpm_config.json" -e 0 -l 10  -z "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_ddr_pym0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_ddr_pym0_1080p/vpm_config.json" -r 10  -z "/app/bin/vps/vpm/res/1080p.yuv" -M 15\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.multi_pym_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_ddr_pym0_gdc_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_ddr_pym0_gdc_1080p/vpm_config.json" -G 0 -e 0 -r 10  -z "/app/bin/vps/vpm/res/1080p.yuv" -M 15\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.multi_pym_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_pym0_common_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym0_common_1080p/vpm_config.json" -r 10  -z "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.pym_common_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_pym0_720p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym0_720p/vpm_config.json"  -z "/app/bin/vps/vpm/res/720p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_pym0_4k"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym0_4k/vpm_config.json"  -r 10 -z "/app/bin/vps/vpm/res/4k.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_pym1_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym1_1080p/vpm_config.json" -r 10 -z "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_pym1_4k"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym1_4k/vpm_config.json"  -r 10 -z "/app/bin/vps/vpm/res/4k.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_pym2_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym2_1080p/vpm_config.json"  -r 10 -z "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_pym2_4k"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym2_4k/vpm_config.json"  -r 10 -z "/app/bin/vps/vpm/res/4k.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_pym0_bpu_720p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym0_bpu_720p/vpm_config.json" -l 10 -e 0 -H 0\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.feedback_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_isp0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_isp0_1080p/vpm_config.json"  -r 10 -c "/app/bin/vps/vpm/cfg/ddr_isp0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmIspTest.vpm_isp_file_raw_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_isp0_pym0_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/ddr_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmIspTest.vpm_isp_pym_file_raw_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ddr_isp_dul"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_isp | grep dul_isp_raw)
	do
		echo "$json"

		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_isp/$json"  -r 10 -M 3 -c "/app/bin/vps/vpm/cfg/ddr_isp/dual_hb_j5dev.json" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmIspTest.multi_isp_raw_feedback

		if [[ $? -ne 0 ]];then
			exit 1
		fi
	done

elif test "$1" == "ddr_isp_quad"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_isp | grep quad_isp_raw)
	do
		echo "$json"

		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_isp/$json"  -r 10 -M 15 -c "/app/bin/vps/vpm/cfg/ddr_isp/quad_hb_j5dev.json" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmIspTest.multi_isp_raw_feedback

		if [[ $? -ne 0 ]];then
			exit 1
		fi
	done

elif test "$1" == "ddr_isp_octu"; then


	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_isp | grep octu_isp_raw)
	do
		echo "$json"

		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_isp/$json"  -r 10 -M 255 -c "/app/bin/vps/vpm/cfg/ddr_isp/octu_hb_j5dev.json" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmIspTest.multi_isp_raw_feedback

		if [[ $? -ne 0 ]];then
			exit 1
		fi
	done

elif test "$1" == "ddr_isp_pym_dul"; then

	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_isp | grep dul_isp_pym)
	do
		echo "$json"

		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_isp/$json"  -r 10 -M 3 -c "/app/bin/vps/vpm/cfg/ddr_isp/dual_hb_j5dev.json" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmIspTest.multi_isp_pym_raw_feedback

		if [[ $? -ne 0 ]];then
			exit 1
		fi
	done

elif test "$1" == "ddr_isp_pym_quad"; then

	echo "Run $1"
	for json in $(ls /app/bin/vps/vpm/cfg/ddr_isp | grep quad_isp_pym)
	do
		echo "$json"

		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_isp/$json"  -r 10 -M 15 -c "/app/bin/vps/vpm/cfg/ddr_isp/quad_hb_j5dev.json" \
		-x ${cam_rstart} -X ${vio_rstart} \
		--gtest_filter=VpmIspTest.multi_isp_pym_raw_feedback

		if [[ $? -ne 0 ]];then
			exit 1
		fi
	done

#==============================================B For Testpattern case===================================================

elif test "$1" == "cim_pym0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_pym0_1080p/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/cim_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_pym0_cond_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_pym0_cond_1080p/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/cim_pym0_cond_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym_cond \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_pym0_preint_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_pym0_preint_1080p/vpm_config.json" -l 10 -c "/app/bin/vps/vpm/cfg/cim_pym0_preint_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym_cond \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cimdma_ddr_pym0_4k"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cimdma_ddr_pym0_4k/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/cimdma_ddr_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cimdma_ddr_pym0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cimdma_ddr_pym0_1080p/vpm_config.json" -r 10 -M 3 -c "/app/bin/vps/vpm/cfg/dul_tp_cimdma_ddr_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cimdma_ddr_pym0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cimdma_ddr_pym0_1080p/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/cimdma_ddr_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cimdma_ddr_isp0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cimdma_ddr_isp0_1080p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cimdma_ddr_isp0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cimdma_ddr_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cimdma_ddr_isp0_pym0_1080p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cimdma_ddr_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cimdma_ddr_isp0_pym0_1080p_manual"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cimdma_ddr_isp0_pym0_1080p_manual/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cimdma_ddr_isp0_pym0_1080p_manual/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cimdma_ddr_isp0_ddr_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cimdma_ddr_isp0_ddr_pym0_1080p/vpm_config.json" -P 1 -c "/app/bin/vps/vpm/cfg/cimdma_ddr_isp0_ddr_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmIspTest.vpm_isp_getdata \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_isp0_pym0_4k/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_isp0_pym0_4k_paththrough"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_isp0_pym0_4k_paththrough/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cim_isp0_pym0_4k_paththrough/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_isp0_pym0_4k_manual"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_isp0_pym0_4k_manual/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cim_isp0_pym0_4k_manual/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_isp0_pym0_1080p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cim_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_isp0_pym0_cond_100ms_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_isp0_pym0_1080p/vpm_config.json" -T 100 -c "/app/bin/vps/vpm/cfg/cim_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym_cond \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_isp0_pym0_cond_minus_100ms_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_isp0_pym0_1080p/vpm_config.json" -T -100 -c "/app/bin/vps/vpm/cfg/cim_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym_cond \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_isp0_pym0_1080p_manual"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_isp0_pym0_1080p_manual/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cim_isp0_pym0_1080p_manual/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_isp0_pym0_720p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_isp0_pym0_720p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cim_isp0_pym0_720p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cim_isp0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cim_isp0_1080p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cim_isp0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cim_isp0_pym0_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cim_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_tp_cim_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cim_isp0_pym0_4k_manual"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cim_isp0_pym0_4k_manual/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_tp_cim_isp0_pym0_4k_manual/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cim_isp0_pym0_4k_1080p_manual"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cim_isp0_pym0_4k_1080p_manual/vpm_config.json" -r 10   -c "/app/bin/vps/vpm/cfg/dul_tp_cim_isp0_pym0_4k_1080p_manual/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cim_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cim_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_tp_cim_isp1_pym1_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cim_isp1_pym1_4k_manual"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cim_isp1_pym1_4k_manual/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_tp_cim_isp1_pym1_4k_manual/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cim_isp1_pym1_4k_1080p_manual"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cim_isp1_pym1_4k_1080p_manual/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_tp_cim_isp1_pym1_4k_1080p_manual/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cim_isp0_pym0_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cim_isp0_pym0_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_tp_cim_isp0_pym0_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cimdma_isp0_pym0_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cimdma_isp0_pym0_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_tp_cimdma_isp0_pym0_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cimdma_isp0_pym0_1080p_manual"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cimdma_isp0_pym0_1080p_manual/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_tp_cimdma_isp0_pym0_1080p_manual/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cimdma_isp0_pym0_1080p_720p_manual"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cimdma_isp0_pym0_1080p_720p_manual/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_tp_cimdma_isp0_pym0_1080p_720p_manual/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_mix_tp_cim_cimdma_isp0_pym0_4k"; then
	echo "Run $1"

	sleep 1
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_mix_tp_cim_cimdma_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_mix_tp_cim_cimdma_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_mix_tp_cim_cimdma_isp0_pym0_4k_manual"; then
	echo "Run $1"

	sleep 1
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_mix_tp_cim_cimdma_isp0_pym0_4k_manual/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_mix_tp_cim_cimdma_isp0_pym0_4k_manual/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_mix_tp_cim_cimdma_isp0_pym0_4k_1080p_manual"; then
	echo "Run $1"

	sleep 1
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_mix_tp_cim_cimdma_isp0_pym0_4k_1080p_manual/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_mix_tp_cim_cimdma_isp0_pym0_4k_1080p_manual/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_tp_cimdma_isp0_ddr_pym0_1080p"; then
	echo "Run $1"

	sleep 1
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_tp_cimdma_isp0_ddr_pym0_1080p/vpm_config.json" -r 10 \
	-c "/app/bin/vps/vpm/cfg/dul_tp_cimdma_isp0_ddr_pym0_1080p/hb_j5dev.json" -M 3 -P 1\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_isp
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_tp_cim_isp0_pym0_fsd"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_tp_cim_isp0_pym0_fsd/vpm_config.json" -r 20  -c "/app/bin/vps/vpm/cfg/quad_tp_cim_isp0_pym0_fsd/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_tp_cim_isp0_pym0_fsd_1080p_manual"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_tp_cim_isp0_pym0_fsd_1080p_manual/vpm_config.json" -r 20  -c "/app/bin/vps/vpm/cfg/quad_tp_cim_isp0_pym0_fsd_1080p_manual/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_tp_mix_cim_cimdma_isp0_1_pym0_1_480p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_tp_mix_cim_cimdma_isp0_1_pym0_1_480p/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/quad_tp_mix_cim_cimdma_isp0_1_pym0_1_480p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_tp_mix_cim_cimdma_isp0_1_pym0_1_1080p_480p_manual"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_tp_mix_cim_cimdma_isp0_1_pym0_1_1080p_480p_manual/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/quad_tp_mix_cim_cimdma_isp0_1_pym0_1_1080p_480p_manual/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_tp_cimdma_isp0_ddr_pym0_1080p"; then
	echo "Run $1"

	sleep 1
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_tp_cimdma_isp0_ddr_pym0_1080p/vpm_config.json" -r 10 \
	-c "/app/bin/vps/vpm/cfg/quad_tp_cimdma_isp0_ddr_pym0_1080p/hb_j5dev.json" -M 15 -P 1\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_isp
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_tp_cim_isp1_ddr_pym1_1080p"; then
	echo "Run $1"

	sleep 1
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_tp_cim_isp1_ddr_pym1_1080p/vpm_config.json" -r 10 \
	-c "/app/bin/vps/vpm/cfg/quad_tp_cim_isp1_ddr_pym1_1080p/hb_j5dev.json" -M 15 -P 1\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_isp
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "octu_tp_mix_cim_cimdma_isp0_1_pym_0_1_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/octu_tp_mix_cim_cimdma_isp0_1_pym_0_1_1080p/vpm_config.json" -r 20  -c "/app/bin/vps/vpm/cfg/octu_tp_mix_cim_cimdma_isp0_1_pym_0_1_1080p/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "octu_tp_cim_isp01_pym01_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/octu_tp_cim_isp01_pym01_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/octu_tp_cim_isp01_pym01_1080p/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "octu_tp_cim_isp01_pym01_1080p_720p_manual"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/octu_tp_cim_isp01_pym01_1080p_720p_manual/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/octu_tp_cim_isp01_pym01_1080p_720p_manual/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "octu_tp_mix_cim_cimdma_isp0_1_pym_0_1_1080p_720p_manual"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/octu_tp_mix_cim_cimdma_isp0_1_pym_0_1_1080p_720p_manual/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/octu_tp_mix_cim_cimdma_isp0_1_pym_0_1_1080p_720p_manual/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "octu_tp_mix_cim_cimdma_isp0_1_ddr_pym_0_1_1080p"; then
	echo "Run $1"

	sleep 1
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/octu_tp_mix_cim_cimdma_isp0_1_ddr_pym_0_1_1080p/vpm_config.json" -r 10 \
	-c "/app/bin/vps/vpm/cfg/octu_tp_mix_cim_cimdma_isp0_1_ddr_pym_0_1_1080p/hb_j5dev.json" -M 255 -P 1\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_isp
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "oxy_8cim_tp_isp01_pym01_8sen_cimdma_isp01_pym01_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/oxy_8cim_tp_isp01_pym01_8sen_cimdma_isp01_pym01_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/oxy_8cim_tp_isp01_pym01_8sen_cimdma_isp01_pym01_1080p/hb_j5dev.json" -M 65535 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "oxy_sen_8cim_isp01_pym01_8cimdma_isp01_pym01_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/oxy_sen_8cim_isp01_pym01_8cimdma_isp01_pym01_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/oxy_sen_8cim_isp01_pym01_8cimdma_isp01_pym01_1080p/hb_j5dev.json" -M 65535 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "oxy_tp_mix_cim_cimdma_isp0_1_pym0_1_1080p_480p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/oxy_tp_mix_cim_cimdma_isp0_1_pym0_1_1080p_480p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/oxy_tp_mix_cim_cimdma_isp0_1_pym0_1_1080p_480p/hb_j5dev.json" -M 65535 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "oxy_tp_mix_cim_cimdma_isp0_1_ddr_pym0_1_1080p_480p"; then
	echo "Run $1"

	sleep 1
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/oxy_tp_mix_cim_cimdma_isp0_1_ddr_pym0_1_1080p_480p/vpm_config.json" -r 10 \
	-c "/app/bin/vps/vpm/cfg/oxy_tp_mix_cim_cimdma_isp0_1_ddr_pym0_1_1080p_480p/hb_j5dev.json" -M 65535 -P 1\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_isp
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

#==============================C For old sensor casse(TODO,REMOVE)======================================================

elif test "$1" == "sen_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_cond_1080p_ar0820"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_cond_1080p_ar0820/vpm_config.json" -T -100 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_cond_1080p_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym_cond \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ar0820_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ar0820_cim_isp0_pym0_4k/vpm_config.json" -r 10 -e 0 -c "cp /app/bin/vps/vpm/cfg/ar0820_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "ar0820_cim_isp0_4k"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ar0820_cim_isp0_4k/vpm_config.json" -r 10 -e 0 -c "/userdata/ngy/ar0820_cim_isp0_4k//hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmIspTest.vpm_isp_getdata \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_720p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_720p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_720p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgraw_rx0_cim_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx0_cim_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx0_cim_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgyuv_rx0_cim_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx0_cim_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx0_cim_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_iduyuv_rx0_cim_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_iduyuv_rx0_cim_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_iduyuv_rx0_cim_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_rx2raw_rx0_cim_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_rx2raw_rx0_cim_isp0_pym0_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx0_rx2raw_rx0_cim_isp0_pym0_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_rx0raw4_rx3_cim_cimdma_isp01_pym01_1280p_sync"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_rx0raw4_rx3_cim_cimdma_isp01_pym01_1280p_sync/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx0_rx0raw4_rx3_cim_cimdma_isp01_pym01_1280p_sync/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgraw_rx0_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx0_cim_isp0_pym0_4k/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx0_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgyuv_rx0_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx0_cim_isp0_pym0_4k/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx0_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgyuv4_rx0_cim_isp0_pym0_1536p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv4_rx0_cim_isp0_pym0_1536p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv4_rx0_cim_isp0_pym0_1536p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_idu0yuv_rx0_cim_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_idu0yuv_rx0_cim_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_idu0yuv_rx0_cim_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_rx2raw_rx0_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_rx2raw_rx0_cim_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx0_rx2raw_rx0_cim_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_vpgraw_rx0_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_vpgraw_rx0_cim_isp0_pym0_4k/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_vpgraw_rx0_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_vpgyuv_rx0_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_vpgyuv_rx0_cim_isp0_pym0_4k/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_vpgyuv_rx0_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_vpgyuv4_rx0_cim_isp0_pym0_1536p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_vpgyuv4_rx0_cim_isp0_pym0_1536p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_vpgyuv4_rx0_cim_isp0_pym0_1536p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_idu1yuv_rx0_cim_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_idu1yuv_rx0_cim_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_idu1yuv_rx0_cim_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_rx3raw_rx0_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_rx3raw_rx0_cim_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx1_rx3raw_rx0_cim_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_rx3raw4_rx0_cimdma_cim_isp10_pym10_1280p_sync"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_rx3raw4_rx0_cimdma_cim_isp10_pym10_1280p_sync/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx1_rx3raw4_rx0_cimdma_cim_isp10_pym10_1280p_sync/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgraw_rx3_cimdma_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx3_cimdma_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx3_cimdma_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgyuv_rx3_cimdma_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx3_cimdma_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx3_cimdma_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_iduyuv_rx3_cimdma_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_iduyuv_rx3_cimdma_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_iduyuv_rx3_cimdma_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgraw_rx3_cimdma_isp0_pym0_1080p_120fps"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx3_cimdma_isp0_pym0_1080p_120fps/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx3_cimdma_isp0_pym0_1080p_120fps/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgraw_rx3_cimdma_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx3_cimdma_isp0_pym0_4k/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgraw_rx3_cimdma_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgyuv_rx3_cimdma_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx3_cimdma_isp0_pym0_4k/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx3_cimdma_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgyuv_rx3_cimdma_isp0_pym0_1281p_emb"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx3_cimdma_isp0_pym0_1281p_emb/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv_rx3_cimdma_isp0_pym0_1281p_emb/hb_j5dev.json" -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_emb_info \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgyuv4_rx3_cimdma_isp0_pym2_1281p_emb"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv4_rx3_cimdma_isp0_pym2_1281p_emb/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv4_rx3_cimdma_isp0_pym2_1281p_emb/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_emb_info \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_vpgyuv4_rx3_cimdma_isp0_pym2_1536p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv4_rx3_cimdma_isp0_pym2_1536p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_vpgyuv4_rx3_cimdma_isp0_pym2_1536p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_idu0yuv_rx3_cimdma_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_idu0yuv_rx3_cimdma_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_idu0yuv_rx3_cimdma_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_idu0yuv_r3_cimdma_pym2_1080p_idutx"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_idu0yuv_r3_cimdma_pym2_1080p_idutx/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx0_idu0yuv_r3_cimdma_pym2_1080p_idutx/hb_j5dev.json" -M 64 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx0_idu0yuv_hdmi_1080p_logo"; then
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/tx0_idu0yuv_hdmi_1080p/vpm_config.json"  -e 0  -r 10800 -c "/app/bin/vps/vpm/cfg/evm/tx0_idu0yuv_hdmi_1080p/hb_j5dev.json" -M 32768 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.evm_multi_streams_set_mipi_tx \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "evm_tx0_idu0yuv_hdmi_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/tx0_idu0yuv_hdmi_1080p/vpm_config.json"  -e 0  -r 40 -c "/app/bin/vps/vpm/cfg/evm/tx0_idu0yuv_hdmi_1080p/hb_j5dev.json" -M 32768 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.evm_multi_streams_set_mipi_tx \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "evm_rx3_tx0_idu0yuv_hdmi_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/rx3_tx0_idu0yuv_hdmi_1080p/vpm_config.json"  -e 0  -r 40 -c "/app/bin/vps/vpm/cfg/evm/rx3_tx0_idu0yuv_hdmi_1080p/hb_j5dev.json" -M 16 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.evm_multi_streams_set_mipi_tx \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "evm_tx0_idu0yuv_hdmi_720p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/tx0_idu0yuv_hdmi_720p/vpm_config.json"  -e 0  -r 40 -c "/app/bin/vps/vpm/cfg/evm/tx0_idu0yuv_hdmi_720p/hb_j5dev.json" -M 32768 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.evm_multi_streams_set_mipi_tx \

elif test "$1" == "tx0_rx0raw_rx3_cimdma_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx0_rx0raw_rx3_cimdma_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx0_rx0raw_rx3_cimdma_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_vpgraw_rx3_cimdma_isp0_pym0_1080p_120fps"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_vpgraw_rx3_cimdma_isp0_pym0_1080p_120fps/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_vpgraw_rx3_cimdma_isp0_pym0_1080p_120fps/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_vpgraw_rx3_cimdma_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_vpgraw_rx3_cimdma_isp0_pym0_4k/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_vpgraw_rx3_cimdma_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_vpgyuv_rx3_cimdma_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_vpgyuv_rx3_cimdma_isp0_pym0_4k/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_vpgyuv_rx3_cimdma_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_vpgyuv4_rx3_cimdma_isp0_pym2_1536p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_vpgyuv4_rx3_cimdma_isp0_pym2_1536p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_vpgyuv4_rx3_cimdma_isp0_pym2_1536p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_idu1yuv_rx3_cimdma_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_idu1yuv_rx3_cimdma_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_idu1yuv_rx3_cimdma_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_idu1yuv_r3_cimdma_pym2_1080p_idutx"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_idu1yuv_r3_cimdma_pym2_1080p_idutx/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx1_idu1yuv_r3_cimdma_pym2_1080p_idutx/hb_j5dev.json" -M 128 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx1_rx1raw_rx3_cimdma_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx1_rx1raw_rx3_cimdma_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx1_rx1raw_rx3_cimdma_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgraw_rx0_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgraw_rx0_cim_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgraw_rx0_cim_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgraw_rx0_cim_isp0_pym0_4k_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgraw_rx0_cim_isp0_pym0_4k_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgraw_rx0_cim_isp0_pym0_4k_1080p/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgrawx_rx0_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgrawx_rx0_cim_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgrawx_rx0_cim_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgyuv_rx0_cim_pym01_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuv_rx0_cim_pym01_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuv_rx0_cim_pym01_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgyuv_rx0_cim_pym01_4k_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuv_rx0_cim_pym01_4k_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuv_rx0_cim_pym01_4k_1080p/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_idu01yuv_rx0_cim_pym01_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_idu01yuv_rx0_cim_pym01_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_idu01yuv_rx0_cim_pym01_1080p/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_idu01yuv_r3_cimdma_pym2_1080p_idutx"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_idu01yuv_r3_cimdma_pym2_1080p_idutx/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/tx/tx01_idu01yuv_r3_cimdma_pym2_1080p_idutx/hb_j5dev.json" -M 192 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgrawyuv_rx0_cim_isp0_pym1_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgrawyuv_rx0_cim_isp0_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgrawyuv_rx0_cim_isp0_pym1_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgyuvraw_rx0_cim_pym0_isp1_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuvraw_rx0_cim_pym0_isp1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuvraw_rx0_cim_pym0_isp1_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgraw_rx3_cimdma_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgraw_rx3_cimdma_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgraw_rx3_cimdma_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgraw_rx3_cimdma_isp0_pym0_4k_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgraw_rx3_cimdma_isp0_pym0_4k_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgraw_rx3_cimdma_isp0_pym0_4k_1080p/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgrawx_rx3_cimdma_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgrawx_rx3_cimdma_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgrawx_rx3_cimdma_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgyuv_rx3_cimdma_pym01_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuv_rx3_cimdma_pym01_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuv_rx3_cimdma_pym01_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgyuv_rx3_cimdma_pym01_4k_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuv_rx3_cimdma_pym01_4k_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuv_rx3_cimdma_pym01_4k_1080p/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_idu01yuv_rx3_cimdma_pym01_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_idu01yuv_rx3_cimdma_pym01_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_idu01yuv_rx3_cimdma_pym01_1080p/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgrawyuv_rx3_cimdma_isp0_pym1_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgrawyuv_rx3_cimdma_isp0_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgrawyuv_rx3_cimdma_isp0_pym1_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "tx01_vpgyuvraw_rx3_cimdma_pym0_isp1_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuvraw_rx3_cimdma_pym0_isp1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/tx/tx01_vpgyuvraw_rx3_cimdma_pym0_isp1_4k/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_pym0_720p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_720p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_720p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_1080p/vpm_config.json"  -e 0  -r 10 -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_4k/vpm_config.json" -l 10 -e 8 -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_pym0_4k/vpm_config.json" -l 10 -e 0 -c "/app/bin/vps/vpm/cfg/dts_cim_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_pym0_1080p/vpm_config.json" -l 10 -e 0 -p 3 -c "/app/bin/vps/vpm/cfg/dts_cim_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_pym1_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_pym1_4k/vpm_config.json" -l 10 -e 0 -c "/app/bin/vps/vpm/cfg/dts_cim_pym1_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_pym1_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_pym1_1080p/vpm_config.json" -l 10 -e 0 -c "/app/bin/vps/vpm/cfg/dts_cim_pym1_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_4k"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_4k/vpm_config.json" -E 1 -r 10 -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_1080p/vpm_config.json" -E 1 -r 10 -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_4k"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_4k/vpm_config.json" -E 1 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_ddr_isp0_pym0_720p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_pym0_720p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_pym0_720p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_ddr_isp0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_1080p/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_1080p/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_pym0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_pym0_1080p/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cimdma_raw_4k"; then



        echo "Run $1"
        ${COMMON_DIR}vpm_gtest -c "/app/bin/vps/vpm/cfg/tp_cimdma_raw_4k/hb_j5dev.json" \
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VinCaseTest.vin_cim_data_raw \
#       --gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_raw_4k_ar0820_sensing"; then



	echo "Run $1"
	${COMMON_DIR}vpm_gtest -c "/app/bin/vps/vpm/cfg/sen_cimdma_raw_4k_ar0820_sensing/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VinCaseTest.vin_cim_data_raw \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_emd_4k_ar0820"; then



        echo "Run $1"
        ${COMMON_DIR}vpm_gtest -c "/app/bin/vps/vpm/cfg/sen_cimdma_emd_4k_ar0820/hb_j5dev.json" -d 1\
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VinCaseTest.vin_cim_data_raw \
#       --gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_emd_isp0_pym0_4k_ar0820"; then

        echo "Run $1"
        ${COMMON_DIR}vpm_gtest -c "/app/bin/vps/vpm/cfg/sen_cimdma_emd_isp0_pym0_4k_ar0820/hb_j5dev.json" -r 10 -v "/app/bin/vps/vpm/cfg/sen_cimdma_emd_isp0_pym0_4k_ar0820/vpm_config.json" -M 1\
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VpmScenarioTest.multi_streams_emb_data \
#       --gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml


elif test "$1" == "sen_cimdma_raw_1080p"; then



	echo "Run $1"
	${COMMON_DIR}vpm_gtest  -c "/app/bin/vps/vpm/cfg/sen_cimdma_raw_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VinCaseTest.vin_cim_data_raw \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cimdma_yuv422_1080p"; then



	echo "Run $1"
	${COMMON_DIR}vpm_gtest -c "/app/bin/vps/vpm/cfg/tp_cimdma_yuv422_1080p/hb_j5dev.json" -r 10 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VinCaseTest.vin_cim_data_yuv \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_1080p_imx390c_gdc"; then



	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/sen_cimdma_1080p_imx390c_gdc/vpm_config.json"  -c "/app/bin/vps/vpm/cfg/evm/sen_cimdma_1080p_imx390c_gdc/hb_j5dev.json" -r 40 -G 0 -D 1 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VinCaseTest.evm_vin_cim_data_yuv_gdc_stitch \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml


elif test "$1" == "sen_cimdma_yuv422_1080p_cond"; then



        echo "Run $1"
        ${COMMON_DIR}vpm_gtest -c "/app/bin/vps/vpm/cfg/sen_cimdma_yuv422_1080p_cond/hb_j5dev.json" -r 10 -M 1\
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VinCaseTest.vin_cim_data_cond_yuv \
#       --gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dual_sen_cimdma_yuv422_1080p_cond"; then



        echo "Run $1"
        ${COMMON_DIR}vpm_gtest -c "/app/bin/vps/vpm/cfg/dual_sen_cimdma_yuv422_1080p_cond/hb_j5dev.json" -r 10 -M 3\
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VinCaseTest.vin_cim_data_cond_yuv \
#       --gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml


#==============================D For single sen input start ============================================================

elif test "$1" == "sen_cim_isp0_pym0_1080p_ar0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1080p_ar0233/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1080p_ar0233/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_1280p_ar0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -S 1 -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1280p_ar0233/vpm_config.json" -e 0 -r 10000 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1280p_ar0233/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_1080p_ar0233_60fps"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1080p_ar0233_60fps/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1080p_ar0233_60fps/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_1280p_ar0233_45fps"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1280p_ar0233_45fps/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1280p_ar0233_45fps/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_4k_ar0820/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_4k_ovx8b"; then

        echo "========= Run $1"
        ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_4k_ovx8b/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_4k_ovx8b/hb_j5dev.json" \
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#       --gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_4k_ovx8b_sizeerr"; then

        echo "========= Run $1"
        ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_4k_ovx8b_sizeerr/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_4k_ovx8b_sizeerr/hb_j5dev.json" \
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#       --gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_ddr_raw_4k_ovx8b_sizeerr"; then



        echo "========= Run $1"
        ${COMMON_DIR}vpm_gtest -c "/app/bin/vps/vpm/cfg/sen_cimdma_raw_4k_ovx8b_sizeerr/hb_j5dev.json" \
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VinCaseTest.vin_cim_data_raw \
#       --gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_ddr_raw_4k_ovx8b"; then



        echo "========= Run $1"
        ${COMMON_DIR}vpm_gtest -c "/app/bin/vps/vpm/cfg/sen_cimdma_raw_4k_ovx8b/hb_j5dev.json" \
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VinCaseTest.vin_cim_data_raw \
#       --gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml


elif test "$1" == "sen_cim_isp1_pym1_1080p_ar0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp1_pym1_1080p_ar0233/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp1_pym1_1080p_ar0233/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp1_pym1_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp1_pym1_4k_ar0820/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp1_pym1_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_ddr_pym0_ddr_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_ddr_pym0_ddr_4k_ar0820/vpm_config.json" -P 1 -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_ddr_pym0_ddr_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_ddr_isp0_pym0_1080p_ar0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_pym0_1080p_ar0233/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_pym0_1080p_ar0233/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_ddr_isp0_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_pym0_4k_ar0820/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_pym0_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_ddr_isp1_pym1_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp1_pym1_4k_ar0820/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp1_pym1_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_ddr_run_pym0_1080p_ar0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_ddr_run_pym0_1080p_ar0233/vpm_config.json" -P 1 -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_ddr_run_pym0_1080p_ar0233/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data

elif test "$1" == "sen_cim_isp0_ddr_run_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_ddr_run_pym0_4k_ar0820/vpm_config.json" -P 1 -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_ddr_run_pym0_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data

elif test "$1" == "sen_cimdma_isp0_ddr_run_pym0_1080p_ar0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cimdma_isp0_ddr_run_pym0_1080p_ar0233/vpm_config.json" -P 1 -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cimdma_isp0_ddr_run_pym0_1080p_ar0233/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data

elif test "$1" == "sen_cimdma_isp0_ddr_run_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cimdma_isp0_ddr_run_pym0_4k_ar0820/vpm_config.json" -P 1 -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cimdma_isp0_ddr_run_pym0_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data
# for single sen input end

#===================================E For multi sen input start ========================================================

elif test "$1" == "dul_sen_cim_isp0_pym0_1280p_ar0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1280p_ar0233/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1280p_ar0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_1080p_ar0233_sync_max9296_sync"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1080p_ar0233_max9296/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_1080p_ar0233_max9296/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_1280p_ar0233_sunny_max9296_sync"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1280p_ar0233_sunny_max9296/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1280p_ar0233_sunny_max9296/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml


elif test "$1" == "dul_sen_cim_isp0_pym0_1280p_ar0233_96712"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1280p_ar0233_96712/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1280p_ar0233_96712/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4k_ar0820/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4k_ar0820/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_isp1_pym1_bpu_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_isp1_pym1_bpu_4k_ar0820/vpm_config.json" -r 30  -e 8 -H 0 -S 1 -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_isp1_pym1_bpu_4k_ar0820/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.multi_vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_4kx1080p_ar0820"; then
	# PIPE0: | 0: 4K (15s)      | (5s) | 2: 1080P (25s)          |
	# PIPE1: | (5s) | 1: 4K (25s)               | (5s) | 3: 1080P (15s) |

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/vpm_config_4k_0.json" -r 15  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/hb_j5dev.json" -C 0 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym &
	sleep 5
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/vpm_config_4k_1.json" -r 25  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/hb_j5dev.json" -C 1 -M 2 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym &
	sleep 15
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/vpm_config_1080p_0.json" -r 25  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/hb_j5dev.json" -C 2 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym &
	sleep 15
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/vpm_config_1080p_1.json" -r 15  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/hb_j5dev.json" -C 3 -M 2 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp0_pym0_4kx1080p_ar0820_0"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/vpm_config_4k_0.json" -r 30  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/hb_j5dev.json" -C 0 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp0_pym0_4kx1080p_ar0820_1"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/vpm_config_4k_1.json" -r 30  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/hb_j5dev.json" -C 1 -M 2 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp0_pym0_4kx1080p_ar0820_2"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/vpm_config_1080p_0.json" -r 30  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/hb_j5dev.json" -C 2 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp0_pym0_4kx1080p_ar0820_3"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/vpm_config_1080p_1.json" -r 30  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4kx1080p_ar0820/hb_j5dev.json" -C 3 -M 2 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp01_pym01_4kx1080p_ar0820"; then
	# PIPE0: | 0: 4K (15s)      | (5s) | 2: 1080P (25s)          |
	# PIPE1: | (5s) | 1: 4K (25s)               | (5s) | 3: 1080P (15s) |

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/vpm_config_4k_0.json" -r 15  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/hb_j5dev.json" -C 0 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym &
	sleep 5
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/vpm_config_4k_1.json" -r 25  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/hb_j5dev.json" -C 1 -M 2 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym &
	sleep 15
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/vpm_config_1080p_0.json" -r 25  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/hb_j5dev.json" -C 2 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym &
	sleep 15
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/vpm_config_1080p_1.json" -r 15  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/hb_j5dev.json" -C 3 -M 2 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp01_pym01_4kx1080p_ar0820_0"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/vpm_config_4k_0.json" -r 30  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/hb_j5dev.json" -C 0 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp01_pym01_4kx1080p_ar0820_1"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/vpm_config_4k_1.json" -r 30  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/hb_j5dev.json" -C 1 -M 2 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp01_pym01_4kx1080p_ar0820_2"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/vpm_config_1080p_0.json" -r 30  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/hb_j5dev.json" -C 2 -M 1 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp01_pym01_4kx1080p_ar0820_3"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/vpm_config_1080p_1.json" -r 30  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp01_pym01_4kx1080p_ar0820/hb_j5dev.json" -C 3 -M 2 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym

elif test "$1" == "dul_sen_cim_isp0_pym0_4k_ar0820_96712"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4k_ar0820_96712/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4k_ar0820_96712/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_sy0820_ws0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_sy0820_ws0233/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_sy0820_ws0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_ws0820_ws0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_ws0233/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_ws0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_ovx8b_ws0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ovx8b_ws0233/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ovx8b_ws0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_sy0820_sy0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_sy0820_sy0233/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_sy0820_sy0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_ws0820_sy0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_sy0233/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_sy0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_ovx8b_sy0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ovx8b_sy0233/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ovx8b_sy0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_sy0820_ovx3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_sy0820_ovx3c/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_sy0820_ovx3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_ws0820_ovx3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_ovx3c/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_ovx3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_ws0820_gax3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_gax3c/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_gax3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_ws0820_ga0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_ga0233/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_ws0820_ga0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp1_pym1_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cimdma_isp0_pym0_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cimdma_isp0_pym0_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cimdma_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cimdma_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_4k_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4k_cimdma_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_4k_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_codec_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_codec_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_codec_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_venc_mx_multi \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cimdma_isp0_pym0_codec_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cimdma_isp0_pym0_codec_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/dul_sen_cimdma_isp0_pym0_codec_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_venc_mx_multi \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "triad_sen_cim_isp0_pym0_1280p_ar0233_96712"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/triad_sen_cim_isp0_pym0_1280p_ar0233_96712/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/triad_sen_cim_isp0_pym0_1280p_ar0233_96712/hb_j5dev.json" -M 7 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "triad_sen_cim_isp0_pym0_codec_1280p"; then

	echo "Run ======$1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/triad_sen_cim_isp0_pym0_codec_1280p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/triad_sen_cim_isp0_pym0_codec_1280p/hb_j5dev.json" -M 7 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_venc_mx_multi \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_pym0_1080p"; then

	echo "file hobot_dev_cim.c +p" > /sys/kernel/debug/dynamic_debug/control
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1080p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml
	echo "file hobot_dev_cim.c -p" > /sys/kernel/debug/dynamic_debug/control

elif test "$1" == "quad_sen_cim_isp0_pym0_fsd"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_fsd/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_fsd/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_pat_cim_isp0_pym0_1280p_96712"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_pat_cim_isp0_pym0_1280p_96712/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_pat_cim_isp0_pym0_1280p_96712/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_pym0_720p_ar0233"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_720p_ar0233/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_720p_ar0233/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_pym0_1280p_sync"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1280p_sync/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1280p_sync/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_pym0_1280p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1280p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1280p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_1280p_sunny"; then
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_1280p_sunny/vpm_config.json" -E 1 -S 1 -r 300000  -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_1280p_sunny/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_pym0_1280p_sunny"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1280p_sunny/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1280p_sunny/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cimdma_isp0_pym0_1280p_sunny"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cimdma_isp0_pym0_1280p_sunny/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_cimdma_isp0_pym0_1280p_sunny/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cimdma_isp1_pym1_1280p_sensing"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cimdma_isp1_pym1_1280p_sensing/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_cimdma_isp1_pym1_1280p_sensing/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_mix_cim_cimdma_isp01_pym01_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_mix_cim_cimdma_isp01_pym01_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_mix_cim_cimdma_isp01_pym01_1080p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_pym0_1080p_one_sen_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1080p_one_sen_cimdma_isp1_pym1_4k/vpm_config.json" -r 10 -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1080p_one_sen_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_pym0_1280p_one_sen_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1280p_one_sen_cimdma_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_1280p_one_sen_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cimdma_pym0_1536p_isx031"; then
    export GDC_MODULE_SKIP=1
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cimdma_pym0_1536p_isx031/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/quad_sen_cimdma_pym0_1536p_isx031/hb_j5dev.json" -M 15 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "quad_sen_cimdma_pym0_1080p_imx390c"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cimdma_pym0_1080p_imx390c/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/quad_sen_cimdma_pym0_1080p_imx390c/hb_j5dev.json" -M 15 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "sen_cimdma_pym2_1080p_imx390c_gdc"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/sen_cimdma_pym2_1080p_imx390c_gdc/vpm_config.json" -r 40 -e 0 -G 0 -D 1 -c "/app/bin/vps/vpm/cfg/evm/sen_cimdma_pym2_1080p_imx390c_gdc/hb_j5dev.json" -M 1 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym_gdc_process \

elif test "$1" == "quad_sen_cimdma_pym2_1536p_isx031_gdc_stitch"; then
    export GDC_MODULE_SKIP=1
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/quad_sen_cimdma_pym2_1536p_isx031_gdc_stitch/vpm_config.json" -r 10 -e 2 -G 0 -c "/app/bin/vps/vpm/cfg/evm/quad_sen_cimdma_pym2_1536p_isx031_gdc_stitch/hb_j5dev.json" -M 15 \
	--gtest_filter=VpmStitchTest.evm_pym_bpu_gdc_stitch_4in1_process  \	

elif test "$1" == "quad_sen_cimdma_pym2_1080p_imx390c_gdc_stitch"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/quad_sen_cimdma_pym2_1080p_imx390c_gdc_stitch/vpm_config.json" -r 40 -e 2 -G 0 -D 1 -c "/app/bin/vps/vpm/cfg/evm/quad_sen_cimdma_pym2_1080p_imx390c_gdc_stitch/hb_j5dev.json" -M 15 \
    --gtest_filter=VpmStitchTest.evm_pym_bpu_gdc_stitch_4in1_process \	

elif test "$1" == "quad_sen_cimdma_1080p_imx390c_gdc_stitch"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/quad_sen_cimdma_1080p_imx390c_gdc_stitch/vpm_config.json" -r 40 -G 0 -D 1 -c "/app/bin/vps/vpm/cfg/evm/quad_sen_cimdma_1080p_imx390c_gdc_stitch/hb_j5dev.json" -M 15 \
    --gtest_filter=VpmStitchTest.evm_gdc_stitch_4in1_process \	

elif test "$1" == "quad_sen_rx0_cim_pym0_1080p_imx390c_gdc_stitch"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/quad_sen_rx0_cim_pym0_1080p_imx390c_gdc_stitch/vpm_config.json" -r 40 -e 2 -G 0 -D 1 -c "/app/bin/vps/vpm/cfg/evm/quad_sen_rx0_cim_pym0_1080p_imx390c_gdc_stitch/hb_j5dev.json" -M 15 \
    --gtest_filter=VpmStitchTest.evm_pym_bpu_gdc_stitch_4in1_process \	
	
elif test "$1" == "quad_sen_rx1_cim_pym0_1080p_imx390c_gdc_stitch"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/quad_sen_rx1_cim_pym0_1080p_imx390c_gdc_stitch/vpm_config.json" -r 40 -e 2 -G 0 -D 1 -c "/app/bin/vps/vpm/cfg/evm/quad_sen_rx1_cim_pym0_1080p_imx390c_gdc_stitch/hb_j5dev.json" -M 15 \
    --gtest_filter=VpmStitchTest.evm_pym_bpu_gdc_stitch_4in1_process \
	
elif test "$1" == "quad_sen_rx3_cimdma_pym2_1080p_imx390c_gdc_stitch"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/quad_sen_rx3_cimdma_pym2_1080p_imx390c_gdc_stitch/vpm_config.json" -r 40 -e 2 -G 0 -D 1 -c "/app/bin/vps/vpm/cfg/evm/quad_sen_rx3_cimdma_pym2_1080p_imx390c_gdc_stitch/hb_j5dev.json" -M 15 \
    --gtest_filter=VpmStitchTest.evm_pym_bpu_gdc_stitch_4in1_process \

elif test "$1" == "quad_sen_4cim_isp0_pym0_1080p_x3c_gdc_stitch"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/quad_sen_4cim_isp0_pym0_1080p_x3c_gdc_stitch/vpm_config.json"  -r 40 -e 2 -G 0 -D 1 -c "/app/bin/vps/vpm/cfg/evm/quad_sen_4cim_isp0_pym0_1080p_x3c_gdc_stitch/hb_j5dev.json" -M 15 \
	--gtest_filter=VpmStitchTest.evm_pym_bpu_gdc_stitch_4in1_process \

elif test "$1" == "sen_cim_isp0_pym0_4k_x8b_gdc"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/sen_cim_isp0_pym0_4k_x8b_gdc/vpm_config.json" -r 40 -e 8 -G 0 -D 1 -c "/app/bin/vps/vpm/cfg/evm/sen_cim_isp0_pym0_4k_x8b_gdc/hb_j5dev.json" -M 1 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym_gdc_process \	
elif test "$1" == "sen_rx1_cim_isp0_pym0_4k_x8b_gdc"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/sen_rx1_cim_isp0_pym0_4k_x8b_gdc/vpm_config.json" -r 40 -e 8 -D 1 -c "/app/bin/vps/vpm/cfg/evm/sen_rx1_cim_isp0_pym0_4k_x8b_gdc/hb_j5dev.json" -M 1 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym_gdc_process

elif test "$1" == "sen_cimdma_pym2_1080p_imx390c_codec_emmc"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/sen_cimdma_pym2_1080p_imx390c_gdc/vpm_config.json" -r 60 -e 0 -c "/app/bin/vps/vpm/cfg/evm/sen_cimdma_pym2_1080p_imx390c_gdc/hb_j5dev.json" -G 0 -M 1 -H 0 -o 0 -j 1 \
    --gtest_filter=VpmCodecTest.evm_mul_vpm_vpu_h265_emmc \
	
elif test "$1" == "sen_cimdma_pym2_1080p_imx390c_codec"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/sen_cimdma_pym2_1080p_imx390c_gdc/vpm_config.json" -r 500 -e 0 -S 0 -c "/app/bin/vps/vpm/cfg/evm/sen_cimdma_pym2_1080p_imx390c_gdc/hb_j5dev.json" -M 1 -H 0 -o 0 -j 1 \
    --gtest_filter=VpmCodecTest.mul_vpm_bpu_vpu_h265_mux \
	
elif test "$1" == "sen_cimdma_pym2_1080p_imx390c_bpu_codec"; then
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/evm/sen_cimdma_pym2_1080p_imx390c_gdc/vpm_config.json" -r 500 -e 2 -S 1 -c "/app/bin/vps/vpm/cfg/evm/sen_cimdma_pym2_1080p_imx390c_gdc/hb_j5dev.json" -M 1 -H 0 -o 1 -j 1 \
    --gtest_filter=VpmCodecTest.evm_mul_vpm_bpu_vpu_h265_mux \
	
elif test "$1" == "sen_cim_fps_ctrl_api"; then



	echo "Run $1"
	${COMMON_DIR}vpm_gtest -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_fps_ctrl_api/hb_j5dev.json"  -M 1\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VinCaseTest.vin_fps_ctl_api \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_fps_ctrl_api"; then



	echo "Run $1"
	${COMMON_DIR}vpm_gtest -r 10 -c "/app/bin/vps/vpm/cfg/sen_cimdma_fps_ctrl_api/hb_j5dev.json"  -M 1\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VinCaseTest.vin_fps_ctl_api \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "octu_sen_4cim_isp0_pym0_4cim_isp1_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/octu_sen_4cim_isp0_pym0_4cim_isp1_pym1_1080p/vpm_config.json" -r 30 -c "/app/bin/vps/vpm/cfg/octu_sen_4cim_isp0_pym0_4cim_isp1_pym1_1080p/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "octu_sen_4cim_isp0_pym0_4cim_isp1_pym1_bpu_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/octu_sen_4cim_isp0_pym0_4cim_isp1_pym1_bpu_1080p/vpm_config.json" -r 30  -e 2 -H 0 -c "/app/bin/vps/vpm/cfg/octu_sen_4cim_isp0_pym0_4cim_isp1_pym1_bpu_1080p/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.multi_vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "octu_sen_4cim_isp0_pym0_4cimdma_isp1_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/octu_sen_4cim_isp0_pym0_4cimdma_isp1_pym1_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/octu_sen_4cim_isp0_pym0_4cimdma_isp1_pym1_1080p/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "octu_sen_4cimdma_isp0_pym0_4cimdma_isp1_pym1_1280p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/octu_sen_4cimdma_isp0_pym0_4cimdma_isp1_pym1_1280p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/octu_sen_4cimdma_isp0_pym0_4cimdma_isp1_pym1_1280p/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "6v_2x0820_30fps_4x0233_30fps"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_2x0820_30fps_4x0233_30fps/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/6v_2x0820_30fps_4x0233_30fps/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "6v_1xsunny0820_1xsensing0820_30fps_4x0233"; then
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_1xsunny0820_1xsensing0820_30fps_4x0233/vpm_config.json" -r 360000 -k 0 -c "/app/bin/vps/vpm/cfg/6v_1xsunny0820_1xsensing0820_30fps_4x0233/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "6v_p0_r12_cimdma_isp1_pym1_5xovx3c_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_p0_r12_cimdma_isp1_pym1_5xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/6v_p0_r12_cimdma_isp1_pym1_5xovx3c_sync/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "6v_p0_r1_cim_isp1_pym1_ovx3c_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_p0_r1_cim_isp1_pym1_ovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/6v_p0_r1_cim_isp1_pym1_ovx3c_sync/hb_j5dev.json" -M 16 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "6v_p1_r0_cim_isp0_pym0_ar0820_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_p1_r0_cim_isp0_pym0_ar0820_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/6v_p1_r0_cim_isp0_pym0_ar0820_sync/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "6v_p0_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_p0_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/6v_p0_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "6v_p0_r1_cim_isp1_pym1_ar0233_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_p0_r1_cim_isp1_pym1_ar0233_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/6v_p0_r1_cim_isp1_pym1_ar0233_sync/hb_j5dev.json" -M 16 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "6v_p1_r0_cim_isp0_pym0_ar0820g_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/6v_p1_r0_cim_isp0_pym0_ar0820g_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/6v_p1_r0_cim_isp0_pym0_ar0820g_sync/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "9v_2x0820_30fps_3x0233_30fps_4xyuv_30fps"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/custom/9v_2x0820_30fps_3x0233_30fps_4xyuv_30fps/vpm_config.json" -r 1 -k 1 -c "/app/bin/vps/vpm/cfg/custom/9v_2x0820_30fps_3x0233_30fps_4xyuv_30fps/hb_j5dev.json" -M 511 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "10v_1x0820_30fps_5x0233_30fps_4xyuv_30fps"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/custom/10v_1x0820_30fps_5x0233_30fps_4xyuv_30fps/vpm_config.json" -e 0 -r 10 \
	-c "/app/bin/vps/vpm/cfg/custom/10v_1x0820_30fps_5x0233_30fps_4xyuv_30fps/hb_j5dev.json" -M 1023 -H 0 -o 0 -j 0\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "10v_1x0820_30fps_5x0233_30fps_4xyuv_30fps_bpu_codec"; then

        echo "Run $1"
        ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/custom/10v_1x0820_30fps_5x0233_30fps_4xyuv_30fps/vpm_config.json" -e 1 -r 10 \
        -c "/app/bin/vps/vpm/cfg/custom/10v_1x0820_30fps_5x0233_30fps_4xyuv_30fps/hb_j5dev.json" -M 1023 -H 1 -o 1023 -j 0 \
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VpmCodecTest.mul_vpm_bpu_vpu_h265_mux \

elif test "$1" == "10v_1x0820_30fps_5x0233_30fps_4xyuv_30fps_gs960x512_bpu_codec"; then

        echo "Run $1"
        ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/custom/10v_1x0820_30fps_5x0233_30fps_4xyuv_30fps_gs960x512_bpu_codec/vpm_config.json" -e 8 -r 10 \
        -c "/app/bin/vps/vpm/cfg/custom/10v_1x0820_30fps_5x0233_30fps_4xyuv_30fps_gs960x512_bpu_codec/hb_j5dev.json" -M 1023 -H 0 -o 1023 -j 0 \
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VpmCodecTest.mul_vpm_bpu_vpu_h265_mux \

elif test "$1" == "6v_1x0820_30fps_5x0233_30fps_gs960x512_bpu_codec"; then

        echo "Run $1"
        ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/custom/6v_1x0820_30fps_5x0233_30fps_gs960x512_bpu_codec/vpm_config.json" -e 8 -r 10 \
        -c "/app/bin/vps/vpm/cfg/custom/6v_1x0820_30fps_5x0233_30fps_gs960x512_bpu_codec/hb_j5dev.json" -M 63 -H 0 -o 63 -j 0 \
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VpmCodecTest.mul_vpm_bpu_vpu_h265_mux \

elif test "$1" == "4v_4xyuv_30fps_gs960x512_bpu_codec"; then

        echo "Run $1"
        ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/custom/4v_4xyuv_30fps_gs960x512_bpu_codec/vpm_config.json" -e 8 -r 10 \
        -c "/app/bin/vps/vpm/cfg/custom/4v_4xyuv_30fps_gs960x512_bpu_codec/hb_j5dev.json" -M 15 -H 0 -o 15 -j 0 \
        -x ${cam_rstart} -X ${vio_rstart} \
        --gtest_filter=VpmCodecTest.mul_vpm_bpu_vpu_h265_mux \

elif test "$1" == "mp_6v_realyuv_1x0820_30fps_5x0233_30fps"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/custom/mp_6v_realyuv_1x0820_30fps_5x0233_30fps/vpm_config.json" -e 0 -r 36000 \
	-c "/app/bin/vps/vpm/cfg/custom/mp_6v_realyuv_1x0820_30fps_5x0233_30fps/hb_j5dev.json" -M 1008 -H 0 -o 0 -j 0\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "mp_in_10v_restart_quad_sen_cimdma_pym0_1536p_isx031"; then

    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/custom/mp_in_10v_restart_quad_sen_cimdma_pym0_1536p_isx031/vpm_config.json" -r 10 -k 0 \
	-c "/app/bin/vps/vpm/cfg/custom/mp_in_10v_restart_quad_sen_cimdma_pym0_1536p_isx031/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "matrix_p2r0r1r2_6v_sen_cim_cimdma_isp0_isp1_pym0_pym1_4k_ovx8b_1280p_ovx3c"; then

    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0r1r2_6v_sen_cim_cimdma_isp0_isp1_pym0_pym1_4k_ovx8b_1280p_ovx3c/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0r1r2_6v_sen_cim_cimdma_isp0_isp1_pym0_pym1_4k_ovx8b_1280p_ovx3c/hb_j5dev.json" -M 63 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "matrix_p2r0r1r2_6v_sen_cim_cimdma_isp0_isp1_pym0_pym1_4k_sy0820_1280p_ovx3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0r1r2_6v_sen_cim_cimdma_isp0_isp1_pym0_pym1_4k_sy0820_1280p_ovx3c/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0r1r2_6v_sen_cim_cimdma_isp0_isp1_pym0_pym1_4k_sy0820_1280p_ovx3c/hb_j5dev.json" -M 63 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "matrix_p2r2_quad_sen_cimdma_isp0_1536p_isx031"; then
    export GDC_MODULE_SKIP=1
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cimdma_isp0_1536p_isx031/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cimdma_isp0_1536p_isx031/hb_j5dev.json" -M 15 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "matrix_p2r2_dual_sen_cimdma_isp0_1536p_isx031"; then
    export GDC_MODULE_SKIP=1
    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_dual_sen_cimdma_isp0_1536p_isx031/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_dual_sen_cimdma_isp0_1536p_isx031/hb_j5dev.json" -M 3 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "matrix_p2r0r1_dul_sen_cim_isp0_pym0_4k_sync"; then

    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0r1_dul_sen_cim_isp0_pym0_4k_sync/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0r1_dul_sen_cim_isp0_pym0_4k_sync/hb_j5dev.json" -M 3 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "matrix_p2r2_quad_sen_cim_cimdma_isp1_pym1_1280p_ovx3c"; then

    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cim_cimdma_isp1_pym1_1280p_ovx3c/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cim_cimdma_isp1_pym1_1280p_ovx3c/hb_j5dev.json" -M 3 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "matrix_p2r0r1_dul_sen_cim_isp0_pym0_4k_ovx8b_1280p_ovx3c"; then

    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0r1_dul_sen_cim_isp0_pym0_4k_ovx8b_1280p_ovx3c/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0r1_dul_sen_cim_isp0_pym0_4k_ovx8b_1280p_ovx3c/hb_j5dev.json" -M 3 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "matrix_p2r1r2_5v_sen_cim_isp0_pym0_1280p_sync"; then

    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1r2_5v_sen_cim_isp0_pym0_1280p_sync/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1r2_5v_sen_cim_isp0_pym0_1280p_sync/hb_j5dev.json" -M 31 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "matrix_p2r2_quad_sen_cimdma_isp0_pym0_1280p_sync"; then

    echo "========= Run $1"
    ${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cimdma_isp0_pym0_1280p_sync/vpm_config.json" -r 1 -k 0 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cimdma_isp0_pym0_1280p_sync/hb_j5dev.json" -M 15 \
    --gtest_filter=VpmScenarioTest.multi_streams_pym \

elif test "$1" == "5v_sen_4cim_isp0_pym0_1cimdma_isp1_pym1_bpu_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/5v_sen_4cim_isp0_pym0_1cimdma_isp1_pym1_bpu_1080p/vpm_config.json" -r 10 -e 2 -H 0 -c "/app/bin/vps/vpm/cfg/5v_sen_4cim_isp0_pym0_1cimdma_isp1_pym1_bpu_1080p/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.multi_vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

#===============================F For old multi sen case ===============================================================


elif test "$1" == "dul_mix_tp_cim_cimdma_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_mix_tp_cim_cimdma_isp0_pym0_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/dul_mix_tp_cim_cimdma_isp0_pym0_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_cim_isp0_pym0_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_cim_isp0_pym0_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/quad_cim_isp0_pym0_1080p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_pym0_bpu_gdc_stitch_1080p"; then
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_bpu_gdc_stitch_1080p/vpm_config.json" -e 0 -r 30 -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_bpu_gdc_stitch_1080p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmStitchTest.pym_bpu_gdc_stitch_4in1_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_sen_cim_isp0_pym0_codec_sender_1080p"; then
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_codec_sender_1080p/vpm_config.json" -e 0 -r 30 -c "/app/bin/vps/vpm/cfg/quad_sen_cim_isp0_pym0_codec_sender_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_vpu_h265_sender_multi \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cimdma_ddr_pym0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest   -l 10 -v "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_pym0_1080p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_pym0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_pym0_ddr_bpu_720p"; then
	echo "Run $1"

	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_720p/vpm_config.json" -e 0 -l 10 -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_720p/hb_j5dev.json"\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_pym0_ddr_bpu_1080p"; then
	echo "Run $1"

	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_1080p/vpm_config.json" -e 0 -l 10 -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_1080p/hb_j5dev.json"\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_pym0_ddr_bpu_4k"; then
	echo "Run $1"

	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_4k/vpm_config.json" -e 8  -r 10 -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_4k/hb_j5dev.json"\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_ddr_bpu_1080p"; then
	echo "Run $1"

	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_bpu_1080p/vpm_config.json" -e 0 -l 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_bpu_1080p/hb_j5dev.json"\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_ddr_bpu_4k"; then
	echo "Run $1"

	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_bpu_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_bpu_4k/hb_j5dev.json"\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_dma_isp0_pym0_ddr_bpu_4k"; then
	echo "Run $1"

	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_dma_isp0_pym0_ddr_bpu_4k/vpm_config.json" -e 8 -l 10 -c "/app/bin/vps/vpm/cfg/sen_cim_dma_isp0_pym0_ddr_bpu_4k/hb_j5dev.json"\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_gdc_stitch_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_gdc_stitch_1080p/vpm_config.json" -r 10 -M 15 -G 0 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmStitchTest.gdc_stitch_4in1_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_gdc_stitch_896x400"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_gdc_stitch_1080p/vpm_config.json" -r 10000 -M 15 -G 0 -w "/app/bin/vps/vpm/res/1920_1536_nv12.yuv" \
	-s "/app/bin/vps/vpm/cfg/stitch_cfg/stitch_four_896_400_in_one_896_896.json" -U "/app/bin/vps/vpm/res/1920x1536_to_896x400_0_affine.json"          \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmStitchTest.gdc_stitch_4in1_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "quad_dts_cimdma_gdc_mix"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_dts_cimdma_gdc_mix/vpm_config.json" -r 10 -M 15 -G 0 -e 0  -l 10  -c "/app/bin/vps/vpm/cfg/quad_dts_cimdma_gdc_mix/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmGdcTest.quad_dts_cimdma_gdc_mix \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml


elif test "$1" == "sen_cimdma_ddr_isp0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_1080p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/sen_cimdma_ddr_isp0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp1_pym1_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp1_pym1_4k/vpm_config.json" -c "/app/bin/vps/vpm/cfg/dts_cim_isp1_pym1_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_dol2_cim_isp0_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_dol2_cim_isp0_1080p/vpm_config.json" -c "/app/bin/vps/vpm/cfg/sen_dol2_cim_isp0_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_ddr_codec_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_codec_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_codec_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_jenc_emmc \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_ddr_codec_h265_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_codec_h265_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_codec_h265_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_vpu_h265_enc_dec_display \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_pym0_ddr_codec_h265_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_codec_h265_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_codec_h265_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_vpu_h265_enc_dec_display \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "file_codec_pym0_bpu_704p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/file_codec_pym0_bpu_704p/vpm_config.json" -e 0 -r 10 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_vpu_file_decodec_pym_bpu \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_pym0_ddr_bpu_codec_h265_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_bpu_codec_h265_1080p/vpm_config.json" -e 0 -l 10 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_bpu_codec_h265_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_bpu_vpu_h265_enc_dec_display \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_pym0_ddr_bpu_codec_h265_mux_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_bpu_codec_h265_mux_1080p/vpm_config.json" -e 0 -l 10 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_bpu_codec_h265_mux_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_bpu_vpu_h265_mux \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_pym0_ddr_bpu_codec_h265_mux_4k"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_bpu_codec_h265_mux_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_bpu_codec_h265_mux_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_bpu_vpu_h265_mux \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_pym0_ddr_bpu_codec_h265_mux_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_codec_h265_mux_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_codec_h265_mux_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_bpu_vpu_h265_mux \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_isp0_pym0_ddr_bpu_codec_h265_mux_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_codec_h265_mux_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/dts_cim_isp0_pym0_ddr_bpu_codec_h265_mux_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_bpu_vpu_h265_mux \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "0820_cim_isp0_pym0_ddr_bpu_codec_h265_mux_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/0820_cim_isp0_pym0_ddr_bpu_codec_h265_mux_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/0820_cim_isp0_pym0_ddr_bpu_codec_h265_mux_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_bpu_vpu_h265_mux \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_ddr_codec_h265_sender_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_codec_h265_sender_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_codec_h265_sender_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_vpu_h265_sender \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_isp0_pym0_ddr_codec_record_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_codec_record_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_isp0_pym0_ddr_codec_record_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_venc_mx \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "sen_cim_pym0_ddr_codec_record_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_codec_record_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/sen_cim_pym0_ddr_codec_record_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_venc_mx \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cim_isp0_pym0_codec"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_codec/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/dul_sen_cim_isp0_pym0_codec/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_venc_mx_multi \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dul_sen_cimdma_isp0_pym0_codec"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dul_sen_cimdma_isp0_pym0_codec/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/dul_sen_cimdma_isp0_pym0_codec/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_venc_mx_multi \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "dts_cim_pym0_ddr_bpu_1080p"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/dts_cim_pym0_ddr_bpu_1080p/vpm_config.json" -e 0 -c "/app/bin/vps/vpm/cfg/dts_cim_pym0_ddr_bpu_1080p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_abd0_sen_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_abd0_sen_cim_isp0_pym0_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_abd0_sen_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_abd0_sen_cim_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_abd0_sen_cim_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_abd0_sen_cim_isp0_pym0_1280p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_c0_sen_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_c0_sen_cim_isp0_pym0_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_c0_sen_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_c0_sen_cim_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_c0_sen_cim_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_c0_sen_cim_isp0_pym0_1280p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_b1_sen_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_b1_sen_cim_isp0_pym0_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_b1_sen_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml


elif test "$1" == "matrix_b1_sen_cim_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_b1_sen_cim_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_b1_sen_cim_isp0_pym0_1280p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_ac1_sen_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_ac1_sen_cim_isp0_pym0_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_ac1_sen_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml


elif test "$1" == "matrix_ac1_sen_cim_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_ac1_sen_cim_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_ac1_sen_cim_isp0_pym0_1280p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_b2_quad_sen_cimdma_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_b2_quad_sen_cimdma_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_b2_quad_sen_cimdma_isp0_pym0_1280p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_b3_quad_sen_cimdma_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_b3_quad_sen_cimdma_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_b3_quad_sen_cimdma_isp0_pym0_1280p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_c2_quad_sen_cimdma_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_c2_quad_sen_cimdma_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_c2_quad_sen_cimdma_isp0_pym0_1280p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_c3_quad_sen_cimdma_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_c3_quad_sen_cimdma_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_c3_quad_sen_cimdma_isp0_pym0_1280p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r0_sen_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0_sen_cim_isp0_pym0_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0_sen_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r0_sen_cim_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0_sen_cim_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0_sen_cim_isp0_pym0_1280p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r1_sen_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1_sen_cim_isp0_pym0_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1_sen_cim_isp0_pym0_4k/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r1_sen_cim_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1_sen_cim_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1_sen_cim_isp0_pym0_1280p/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r0_dul_sen_cim_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0_dul_sen_cim_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0_dul_sen_cim_isp0_pym0_1280p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r0_dul_sen_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0_dul_sen_cim_isp0_pym0_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r0_dul_sen_cim_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r1_dul_sen_cim_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1_dul_sen_cim_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1_dul_sen_cim_isp0_pym0_1280p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r1_dul_sen_cim_isp0_pym0_4k"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1_dul_sen_cim_isp0_pym0_4k/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r1_dul_sen_cim_isp0_pym0_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r2_quad_sen_cimdma_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cimdma_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cimdma_isp0_pym0_1280p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r2_quad_sen_cimdma_isp0_pym0_1280p_ovx3c"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cimdma_isp0_pym0_1280p_ovx3c/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r2_quad_sen_cimdma_isp0_pym0_1280p_ovx3c/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r3_quad_sen_cimdma_isp0_pym0_1280p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r3_quad_sen_cimdma_isp0_pym0_1280p/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r3_quad_sen_cimdma_isp0_pym0_1280p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "matrix_p2r3_quad_sen_cimdma_isp0_pym0_1280p_ovx3c"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/matrix/matrix_p2r3_quad_sen_cimdma_isp0_pym0_1280p_ovx3c/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/matrix/matrix_p2r3_quad_sen_cimdma_isp0_pym0_1280p_ovx3c/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

#-------------------------- for GALAXY --------------------------
elif test "$1" == "galaxy_sen_r0_cim_isp0_pym0_ddr_bpu_ar0820"; then
	echo "Run $1"
	export GDC_MODULE_SKIP=1
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_sen_r0_cim_isp0_pym0_ddr_bpu_ar0820/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_sen_r0_cim_isp0_pym0_ddr_bpu_ar0820/hb_j5dev.json"\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.vpm_pym_bpu_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_sen_r0_cim_isp0_pym0_ar0820_tx"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_sen_r0_cim_isp0_pym0_ar0820_tx/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_sen_r0_cim_isp0_pym0_ar0820_tx/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	-D 0x0855 -y /app/bin/vps/vpm/cfg/galaxy/galaxy_idu.json \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_sen_r0_cim_isp0_pym0_ar0820_tx_new"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_sen_r0_cim_isp0_pym0_ar0820_tx_new/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_sen_r0_cim_isp0_pym0_ar0820_tx_new/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	-D 0x0855 -y /app/bin/vps/vpm/cfg/galaxy/galaxy_idu.json \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_dul_r01_cim_isp01_pym01_ar0820_ovx3c"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r01_cim_isp01_pym01_ar0820_ovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r01_cim_isp01_pym01_ar0820_ovx3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_dul_r0_cim_isp0_pym0_ar0820_ovx3c"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r0_cim_isp0_pym0_ar0820_ovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r0_cim_isp0_pym0_ar0820_ovx3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_dul_r1_cim_isp1_pym1_ar0820_ovx3c"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r1_cim_isp1_pym1_ar0820_ovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r1_cim_isp1_pym1_ar0820_ovx3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_dul_r01_cim_isp01_pym01_ar0820_ar0233"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r01_cim_isp01_pym01_ar0820_ar0233/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r01_cim_isp01_pym01_ar0820_ar0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_dul_r01_cim_isp01_pym01_ar0820_ar0233_new"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r01_cim_isp01_pym01_ar0820_ar0233_new/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r01_cim_isp01_pym01_ar0820_ar0233_new/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_dul_r0_cim_isp0_pym0_ar0820_ar0233"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r0_cim_isp0_pym0_ar0820_ar0233/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r0_cim_isp0_pym0_ar0820_ar0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_dul_r1_cim_isp1_pym1_ar0820_ar0233"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r1_cim_isp1_pym1_ar0820_ar0233/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_dul_r1_cim_isp1_pym1_ar0820_ar0233/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_quad_r2_cimdma_isp1_pym1_ovx3c"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_quad_r2_cimdma_isp1_pym1_ovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_quad_r2_cimdma_isp1_pym1_ovx3c/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_quad_r3_cimdma_pym2_1280p_hu"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_quad_r3_cimdma_pym2_1280p_hu/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_quad_r3_cimdma_pym2_1280p_hu/hb_j5dev.json" -M 960 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_quad_r3_cimdma_pym2_1280p_hu_emb"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_quad_r3_cimdma_pym2_1280p_hu_emb/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_quad_r3_cimdma_pym2_1280p_hu_emb/hb_j5dev.json" -M 960 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_emb_info \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_quad_r3_cimdma_pym2_1080p_hu"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_quad_r3_cimdma_pym2_1080p_hu/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_quad_r3_cimdma_pym2_1080p_hu/hb_j5dev.json" -M 960 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/hb_j5dev.json" -M 192 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p_dv"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/vpm_config.json" -m 30 -e 9 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/hb_j5dev.json" -M 192 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p_dvshow"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/vpm_config.json" -m 30 -e 9 -S 1 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/hb_j5dev.json" -M 192 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p_dvx"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/vpm_config.json" -m 30 -e 10 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/hb_j5dev.json" -M 192 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p_dvxshow"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/vpm_config.json" -m 30 -e 10 -S 1 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym2_1080p/hb_j5dev.json" -M 192 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t01_vpgyuv_r3_cimdma_pym12_4k"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym12_4k/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_vpgyuv_r3_cimdma_pym12_4k/hb_j5dev.json" -M 192 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t01_idu01yuv_r3_cimdma_pym2_1080p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_idu01yuv_r3_cimdma_pym2_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_idu01yuv_r3_cimdma_pym2_1080p/hb_j5dev.json" -M 192 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t01_idu01yuv_r3_cimdma_pym2_1080p_idutx"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_idu01yuv_r3_cimdma_pym2_1080p_idutx/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t01_idu01yuv_r3_cimdma_pym2_1080p_idutx/hb_j5dev.json" -M 192 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t0_vpgyuv4_r3_cimdma_pym2_1281p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_vpgyuv4_r3_cimdma_pym2_1281p/vpm_config.json"  -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_vpgyuv4_r3_cimdma_pym2_1281p/hb_j5dev.json" -M 960 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t0_vpgyuv4_r3_cimdma_pym2_1281p_emb"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_vpgyuv4_r3_cimdma_pym2_1281p_emb/vpm_config.json"  -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_vpgyuv4_r3_cimdma_pym2_1281p_emb/hb_j5dev.json" -M 960 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_emb_info \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t0_vpgyuv4_r3_cimdma_pym2_1536p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_vpgyuv4_r3_cimdma_pym2_1536p/vpm_config.json"  -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_vpgyuv4_r3_cimdma_pym2_1536p/hb_j5dev.json" -M 960 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t0_idu0yuv_r3_cimdma_pym2_1080p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_idu0yuv_r3_cimdma_pym2_1080p/vpm_config.json"  -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_idu0yuv_r3_cimdma_pym2_1080p/hb_j5dev.json" -M 64 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t0_idu0yuv_r3_cimdma_pym2_1080p_idutx"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_idu0yuv_r3_cimdma_pym2_1080p_idutx/vpm_config.json"  -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t0_idu0yuv_r3_cimdma_pym2_1080p_idutx/hb_j5dev.json" -M 64 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t1_idu1yuv_r3_cimdma_pym2_1080p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t1_idu1yuv_r3_cimdma_pym2_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t1_idu1yuv_r3_cimdma_pym2_1080p/hb_j5dev.json" -M 128 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_t1_idu1yuv_r3_cimdma_pym2_1080p_idutx"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_t1_idu1yuv_r3_cimdma_pym2_1080p_idutx/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_t1_idu1yuv_r3_cimdma_pym2_1080p_idutx/hb_j5dev.json" -M 128 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_4v_t01_idu01yuv_r013_cim_cimdma_isp01_pym012_4k_1080p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_4v_t01_idu01yuv_r013_cim_cimdma_isp01_pym012_4k_1080p/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_4v_t01_idu01yuv_r013_cim_cimdma_isp01_pym012_4k_1080p/hb_j5dev.json" -M 240 \
	-D 0x485f -y /app/bin/vps/vpm/cfg/galaxy/galaxy_idu.json \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_4v_t01_idu01yuv_r013_cim_cimdma_isp01_pym012_4k_1080p_api"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_4v_t01_idu01yuv_r013_cim_cimdma_isp01_pym012_4k_1080p/vpm_config.json" -D 0x4853 -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_4v_t01_idu01yuv_r013_cim_cimdma_isp01_pym012_4k_1080p/hb_j5dev.json" -M 240 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_dv"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c/vpm_config.json" -e 0 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_dvshow"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c/vpm_config.json" -e 8 -S 1 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_new"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_new/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_new/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_new_dv"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_new/vpm_config.json" -e 0 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_new/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_new_dvshow"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_new/vpm_config.json" -e 8 -S 1 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_new/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r12_cimdma_isp1_pym1_5xovx3c"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_5xovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_5xovx3c/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r0_cim_isp0_pym0_ar0820"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r12_cimdma_isp1_pym1_5xovx3c_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_5xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_5xovx3c_sync/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r0_cim_isp0_pym0_ar0820_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820_sync/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r12_cimdma_isp1_pym1_5xovx3c_sync_vcs"; then
	export GDC_MODULE_SKIP=1
	[ `cat /sys/module/hobot_cim/parameters/vio_mp_en` = "0" ] && echo "$1: vio_mipi_en=0 error" && exit 1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r0_cim_isp0_pym0_ar0820_sync_vcs"; then
	export GDC_MODULE_SKIP=1
	[ `cat /sys/module/hobot_cim/parameters/vio_mp_en` = "0" ] && echo "$1: vio_mipi_en=0 error" && exit 1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_5xovx3c_sync/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_dv"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c/vpm_config.json" -e 0 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_dvshow"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c/vpm_config.json" -e 8 -S 1 -r 10000000 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r0_cim_isp0_pym0_ar0820g"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820g/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820g/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync_new"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync_new/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync_new/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync_new"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync_new/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync_new/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync_new"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync_new/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync_new/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync_idu01"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync_idu01/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync/hb_j5dev.json" -M 63 \
	-x ${cam_rstart} -X ${vio_rstart} \
	-D 0x485f -y /app/bin/vps/vpm/cfg/galaxy/galaxy_idu.json \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync_idu1"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync_idu1/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync_idu1/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	-D 0x400a -y /app/bin/vps/vpm/cfg/galaxy/galaxy_idu.json \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync_idu0"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync_idu0/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync_idu0/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	-D 0x0855 -y /app/bin/vps/vpm/cfg/galaxy/galaxy_idu.json \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r12_cimdma_isp1_pym1_ar0233_4xovx3c_sync_vcs"; then
	export GDC_MODULE_SKIP=1
	[ `cat /sys/module/hobot_cim/parameters/vio_mp_en` = "0" ] && echo "$1: vio_mipi_en=0 error" && exit 1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_6v_r0_cim_isp0_pym0_ar0820g_sync_vcs"; then
	export GDC_MODULE_SKIP=1
	[ `cat /sys/module/hobot_cim/parameters/vio_mp_en` = "0" ] && echo "$1: vio_mipi_en=0 error" && exit 1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_6v_r012_cim_cimdma_isp01_pym01_ar0820_ar0233_4xovx3c_sync/hb_j5dev.json" -M 32 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_8v_t0_idu0yuv_r03_cim_cimdma_isp0_pym02_4k_1080p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_8v_t0_idu0yuv_r03_cim_cimdma_isp0_pym02_4k_1080p/vpm_config.json" -D 0x801 -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_8v_t0_idu0yuv_r03_cim_cimdma_isp0_pym02_4k_1080p/hb_j5dev.json" -M 3 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_8v_t1_idu1yuv_r13_cim_cimdma_isp0_pym02_1280p_1080p"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_8v_t1_idu1yuv_r13_cim_cimdma_isp0_pym02_1280p_1080p/vpm_config.json" -D 0x2002 -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_8v_t1_idu1yuv_r13_cim_cimdma_isp0_pym02_1280p_1080p/hb_j5dev.json" -M 12 \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_8v_quad_r2_cimdma_isp1_pym1_ovx3c"; then
	export GDC_MODULE_SKIP=1
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_8v_quad_r2_cimdma_isp1_pym1_ovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_8v_quad_r2_cimdma_isp1_pym1_ovx3c/hb_j5dev.json" -M 240 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_5xovx3c_hu"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_5xovx3c_hu/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_5xovx3c_hu/hb_j5dev.json" -M 1023 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_5xovx3c_t0"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_5xovx3c_t0/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_5xovx3c_t0/hb_j5dev.json" -M 1023 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_hu"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_hu/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_hu/hb_j5dev.json" -M 1023 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_t0"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_t0/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_t0/hb_j5dev.json" -M 1023 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_hu_new"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_hu_new/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_hu_new/hb_j5dev.json" -M 1023 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_t0_new"; then
	export GDC_MODULE_SKIP=1
	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_t0_new/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/galaxy/galaxy_10v_r0123_cim_cimdma_isp01_pym012_ar0820_ar0233_4xovx3c_t0_new/hb_j5dev.json" -M 1023 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

#-------------------------- for CICD A --------------------------

elif test "$1" == "cicd_sen_cim_isp0_4k"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_4k_ar0820/vpm_config.json" -E 1 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cim_isp0_pym0_ddr_bpu_4k_ar0820_paththrough"; then
	echo "Run $1"

	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_pym0_ddr_bpu_4k_ar0820_paththrough/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_pym0_ddr_bpu_4k_ar0820_paththrough/hb_j5dev.json"\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmBpuTest.vpm_pym_bpu_process \

elif test "$1" == "cicd_sen_cim_isp0_pym0_ddr_bpu_codec_4k_h265_mux_ar0820"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_pym0_ddr_bpu_codec_4k_h265_mux_ar0820/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_pym0_ddr_bpu_codec_4k_h265_mux_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_bpu_vpu_h265_mux \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_isp0_pym0_bpu_gdc_stitch_1080p_ovx3c_manual"; then
	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_bpu_gdc_stitch_1080p_ovx3c_manual/vpm_config.json" -e 0 -r 30 -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_bpu_gdc_stitch_1080p_ovx3c_manual/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmStitchTest.pym_bpu_gdc_stitch_4in1_process \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cim_isp0_ddr_run_pym0_1080p_x3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_run_pym0_1080p_x3c/vpm_config.json" -P 1 -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_run_pym0_1080p_x3c/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data

elif test "$1" == "cicd_sen_cim_isp0_ddr_run_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_run_pym0_4k_ar0820/vpm_config.json" -P 1 -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_run_pym0_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data

elif test "$1" == "cicd_sen_cim_isp0_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_pym0_4k_ar0820/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_pym0_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cim_isp0_pym0_1080p_x3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_pym0_1080p_x3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_pym0_1080p_x3c/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cim_isp1_pym1_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp1_pym1_4k_ar0820/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp1_pym1_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cim_isp1_pym1_1080p_x3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp1_pym1_1080p_x3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp1_pym1_1080p_x3c/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_pym_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cim_isp0_ddr_pym0_ddr_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_pym0_ddr_4k_ar0820/vpm_config.json" -P 1 -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_pym0_ddr_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cimdma_ddr_isp0_1080p_x3c"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_ddr_isp0_1080p_x3c/vpm_config.json" -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_ddr_isp0_1080p_x3c/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cimdma_isp0_ddr_run_pym0_1080p_x3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_isp0_ddr_run_pym0_1080p_x3c/vpm_config.json" -P 1 -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_isp0_ddr_run_pym0_1080p_x3c/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data

elif test "$1" == "cicd_sen_cimdma_isp0_ddr_run_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_isp0_ddr_run_pym0_4k_ar0820/vpm_config.json" -P 1 -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_isp0_ddr_run_pym0_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.vpm_scenario_isp_data

elif test "$1" == "cicd_sen_cimdma_ddr_isp0_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_ddr_isp0_pym0_4k_ar0820/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_ddr_isp0_pym0_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cimdma_ddr_isp0_pym0_1080p_x3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_ddr_isp0_pym0_1080p_x3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_ddr_isp0_pym0_1080p_x3c/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cimdma_ddr_isp1_pym1_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_ddr_isp1_pym1_4k_ar0820/vpm_config.json" -e 8 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_ddr_isp1_pym1_4k_ar0820/hb_j5dev.json" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.vin_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cimdma_isp1_tdmf_pym1_4k_pipe4"; then
	# pipe 4

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_isp1_tdmf_pym1_4k_pipe4/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cimdma_isp1_tdmf_pym1_4k_pipe4/hb_j5dev.json" -M 16 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cim_isp1_passthru_pym1_4k_pipe4"; then
	# pipe 4

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp1_passthru_pym1_4k_pipe4/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp1_passthru_pym1_4k_pipe4/hb_j5dev.json" -M 16 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cim_isp0_pym0_1080p_ovx3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_1080p_ovx3c/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_1080p_ovx3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cimdma_isp0_pym0_1080p_ovx3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cimdma_isp0_pym0_1080p_ovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cimdma_isp0_pym0_1080p_ovx3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cim_isp0_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_4k_ar0820/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_4k_ar0820/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cim_isp1_pym1_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp1_pym1_4k_ar0820/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp1_pym1_4k_ar0820/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cimdma_isp0_pym0_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cimdma_isp0_pym0_4k_ar0820/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cimdma_isp0_pym0_4k_ar0820/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cimdma_isp1_pym1_4k_ar0820"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cimdma_isp1_pym1_4k_ar0820/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cimdma_isp1_pym1_4k_ar0820/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_1080p/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_1080p_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cim_isp0_pym0_4k_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_4k_cimdma_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_4k_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cim_isp0_pym0_codec_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_1080p_ovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cim_isp0_pym0_1080p_ovx3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_venc_mx_multi \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_dul_sen_cimdma_isp0_pym0_codec_1080p"; then

	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cimdma_isp0_pym0_1080p_ovx3c/vpm_config.json" -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/dul_sen_cimdma_isp0_pym0_1080p_ovx3c/hb_j5dev.json" -M 3 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmCodecTest.vpm_venc_mx_multi \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_isp0_pym0_1080p_ovx3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1080p_ovx3c/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1080p_ovx3c/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_isp0_pym0_1280p_ovx3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1280p_ovx3c/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1280p_ovx3c/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cimdma_isp0_pym0_1280p_ovx3c"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_pym0_1280p_ovx3c/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_pym0_1280p_ovx3c/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cimdma_isp0_format_1280p_ovx3c"; then


	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_format_1280p_ovx3c/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_format_1280p_ovx3c/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_isp \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_mix_cim_cimdma_isp01_pym01_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_mix_cim_cimdma_isp01_pym01_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_mix_cim_cimdma_isp01_pym01_1080p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_isp0_pym0_1080p_one_sen_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1080p_one_sen_cimdma_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1080p_one_sen_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_isp0_pym0_1280p_one_sen_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1280p_one_sen_cimdma_isp1_pym1_4k/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1280p_one_sen_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_isp0_tdmf_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k"; then
	# 5 pipes enable lpwm

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_tdmf_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_tdmf_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_isp0_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k"; then
	# 5 pipes enable lpwm

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_isp0_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cimdma_isp0_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k"; then
	# 5 pipes enable lpwm

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cimdma_isp0_tdmf_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k"; then
	# 5 pipes enable lpwm

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_tdmf_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_tdmf_pym0_1280p_one_sen_cim_isp1_passthru_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cimdma_isp0_pym0_1280p_one_sen_cimdma_isp1_tdmf_pym1_4k"; then
	# 5 pipes enable lpwm

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_pym0_1280p_one_sen_cimdma_isp1_tdmf_pym1_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_pym0_1280p_one_sen_cimdma_isp1_tdmf_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cimdma_isp0_tdmf_pym0_1280p_one_sen_cimdma_isp1_tdmf_pym1_4k"; then
	# 5 pipes enable lpwm

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_tdmf_pym0_1280p_one_sen_cimdma_isp1_tdmf_pym1_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cimdma_isp0_tdmf_pym0_1280p_one_sen_cimdma_isp1_tdmf_pym1_4k/hb_j5dev.json" -M 31 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_cimdma_isp0_pym0_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_cimdma_isp0_pym0_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_cimdma_isp0_pym0_4k/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_cimdma_isp0_tdmf_pym0_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_cimdma_isp0_tdmf_pym0_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_cimdma_isp0_tdmf_pym0_4k/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_cimdma_isp1_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_cimdma_isp1_pym1_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_cimdma_isp1_pym1_4k/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_cim_cimdma_isp1_tdmf_pym1_4k"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_cimdma_isp1_tdmf_pym1_4k/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_a/quad_sen_cim_cimdma_isp1_tdmf_pym1_4k/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_sen_cim_isp0_ddr_run_pym0_4k_ar0820_isp_api"; then


	echo "========= Run $1"

	for i in $(seq 1 19)
	do
		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_run_pym0_4k_ar0820/vpm_config.json" \
		-x ${cam_rstart} -X ${vio_rstart} \
		-P 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_run_pym0_4k_ar0820/hb_j5dev.json" \
		-I "/app/bin/vps/vpm/cfg/isp_cfg.json" -M 1 -i `echo $ispcfg_idx | awk -v j="$i" '{print $j}'` \
		-A `echo $static_param | awk -v j="$i" '{print $j}'` -a  `echo $dynamic_param | awk -v j="$i" '{print $j}'` \
		--gtest_filter=VpmIspTest.vpm_isp_getdata
	done

elif test "$1" == "cicd_sen_cim_isp0_ddr_run_pym0_1080p_x3c_isp_api"; then

	#
	echo "========= Run $1"

	for i in $(seq 1 19)
	do
		${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_run_pym0_1080p_x3c/vpm_config.json" \
		-x ${cam_rstart} -X ${vio_rstart} \
		-P 0 -e 0 -r 10 -c "/app/bin/vps/vpm/cfg/cicd_a/sen_cim_isp0_ddr_run_pym0_1080p_x3c/hb_j5dev.json" \
		-I "/app/bin/vps/vpm/cfg/isp_cfg.json" -M 1 -i `echo $ispcfg_idx | awk -v j="$i" '{print $j}'` \
		-A `echo $static_param | awk -v j="$i" '{print $j}'` -a  `echo $dynamic_param | awk -v j="$i" '{print $j}'` \
		--gtest_filter=VpmIspTest.vpm_isp_getdata
	done

elif test "$1" == "cicd_ddr_pym0_1080p_md5"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym0_1080p/vpm_config.json" -e 0 -l 50 -m 1 -z "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_ddr_pym0_1080p_md5"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/quad_ddr_pym0_1080p/vpm_config.json" -r 20 -m 1 -z "/app/bin/vps/vpm/res/1080p.yuv" -M 15\
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.multi_pym_feedback \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_ddr_pym1_1080p_md5"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym1_1080p/vpm_config.json" -r 20 -m 1 -z "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_ddr_pym1_4k_md5"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym1_4k/vpm_config.json"  -r 20 -m 1 -z "/app/bin/vps/vpm/res/4k.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_ddr_pym2_1080p_md5"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym2_1080p/vpm_config.json"  -r 20 -m 1 -z "/app/bin/vps/vpm/res/1080p.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_ddr_pym2_4k_md5"; then


	echo "Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/ddr_pym2_4k/vpm_config.json"  -r 20 -m 1 -z "/app/bin/vps/vpm/res/4k.yuv" \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmPymTest.feedback_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml



#-------------------------- for CICD B --------------------------

elif test "$1" == "cicd_octu_sen_4cim_isp0_pym0_4cim_isp1_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/octu_sen_4cim_isp0_pym0_4cim_isp1_pym1_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/octu_sen_4cim_isp0_pym0_4cim_isp1_pym1_1080p/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_octu_sen_4cim_isp0_pym0_4cimdma_isp1_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/octu_sen_4cim_isp0_pym0_4cimdma_isp1_pym1_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/octu_sen_4cim_isp0_pym0_4cimdma_isp1_pym1_1080p/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_octu_sen_4cimdma_isp0_pym0_4cimdma_isp1_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/octu_sen_4cimdma_isp0_pym0_4cimdma_isp1_pym1_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/octu_sen_4cimdma_isp0_pym0_4cimdma_isp1_pym1_1080p/hb_j5dev.json" -M 255 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_oxy_sen_8cim_isp01_pym01_8cimdma_isp01_pym01_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/oxy_sen_8cim_isp01_pym01_8cimdma_isp01_pym01_1080p/vpm_config.json" -r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/oxy_sen_8cim_isp01_pym01_8cimdma_isp01_pym01_1080p/hb_j5dev.json" -M 65535 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

# 16pipe in 4 process: all manual mode isp-otf-pym:
elif test "$1" == "cicd_quad_sen_4cim_isp0_pym0_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cim_isp0_pym0_1080p/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cim_isp0_pym0_1080p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_4cim_isp1_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cim_isp1_pym1_1080p/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cim_isp1_pym1_1080p/hb_j5dev.json" -M 240 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_4cimdma_isp0_pym0_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cimdma_isp0_pym0_1080p/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cimdma_isp0_pym0_1080p/hb_j5dev.json" -M 3840 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_4cimdma_isp1_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cimdma_isp1_pym1_1080p/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cimdma_isp1_pym1_1080p/hb_j5dev.json" -M 61440 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

# 16pipe in 4 process: all tdmf mode isp-otf-pym:
elif test "$1" == "cicd_quad_sen_4cim_isp0_tdmf_pym0_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cim_isp0_tdmf_pym0_1080p/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cim_isp0_tdmf_pym0_1080p/hb_j5dev.json" -M 15 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_4cim_isp1_tdmf_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cim_isp1_tdmf_pym1_1080p/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cim_isp1_tdmf_pym1_1080p/hb_j5dev.json" -M 240 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_4cimdma_isp0_tdmf_pym0_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cimdma_isp0_tdmf_pym0_1080p/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cimdma_isp0_tdmf_pym0_1080p/hb_j5dev.json" -M 3840 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "cicd_quad_sen_4cimdma_isp1_tdmf_pym1_1080p"; then

	echo "========= Run $1"
	${COMMON_DIR}vpm_gtest -v "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cimdma_isp1_tdmf_pym1_1080p/vpm_config.json" \
	-r 10  -c "/app/bin/vps/vpm/cfg/cicd_b/quad_sen_4cimdma_isp1_tdmf_pym1_1080p/hb_j5dev.json" -M 61440 \
	-x ${cam_rstart} -X ${vio_rstart} \
	--gtest_filter=VpmScenarioTest.multi_streams_pym \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

elif test "$1" == "evm_emmc_vpu_h265_dec_display"; then

	echo "========= Run $1"
	/app/bin/vps/vpm/vio_test_case.sh evm_tx0_idu0yuv_hdmi_1080p
	sleep 5
	${COMMON_DIR}vpm_gtest -V "/app/bin/vps/vpm/res/input_1920x1080_normal.h265" \
	-y "/app/bin/vps/vpm/cfg/idu_cfg/idu_hdmi_1080p.json" \
	--gtest_filter=VpmCodecTest.evm_emmc_vpu_h265_dec_display \
#	--gtest_output=xml:${OUTPUT_DIR_PREFIX}output/vps_$1_test_result.xml

else
	echo "invaild cmd input : $1 "

fi
