#!/bin/bash
# Copyright (c) 2015-2017, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

CONF_FILE=${HOME}/l4t_dfs.conf
RED='\e[0;31m'
GREEN='\e[0;32m'
BLUE='\e[0;34m'
BRED='\e[1;31m'
BGREEN='\e[1;32m'
BBLUE='\e[1;34m'
NC='\e[0m' # No Color

usage()
{
	if [ "$1" != "" ]; then
		echo -e ${RED}"$1"${NC}
	fi

		cat >& 2 <<EOF
Maximize jetson performance by setting static max frequency to CPU, GPU and EMC clocks.
Usage:
jetson_clocks.sh [options]
  options,
  --show             display current settings
  --store [file]     store current settings to a file (default: \${HOME}/l4t_dfs.conf)
  --restore [file]   restore saved settings from a file (default: \${HOME}/l4t_dfs.conf)
  run jetson_clocks.sh without any option to set static max frequency to CPU, GPU and EMC clocks.
EOF

	exit 0
}

restore()
{
	for conf in `cat "${CONF_FILE}"`; do
		file=`echo $conf | cut -f1 -d :`
		data=`echo $conf | cut -f2 -d :`
		case "${file}" in
			/sys/devices/system/cpu/cpu*/online |\
			/sys/kernel/debug/clk/override*/state)
				if [ `cat $file` -ne $data ]; then
					echo "${data}" > "${file}"
				fi
				;;
			/sys/devices/system/cpu/cpu*/cpufreq/scaling_min_freq |\
			/sys/kernel/debug/tegra_cpufreq/*_CLUSTER/cc3/enable)
				echo "${data}" > "${file}" 2>/dev/null
				;;
			*)
				echo "${data}" > "${file}"
				ret=$?
				if [ ${ret} -ne 0 ]; then
					echo "Error: Failed to restore $file"
				fi
				;;
		esac
	done
}

store()
{
	for file in $@; do
		if [ -e "${file}" ]; then
			echo "${file}:`cat ${file}`" >> "${CONF_FILE}"
		fi
	done
}

do_fan()
{
	# Jetson-TK1 CPU fan is always ON.
	if [ "${machine}" = "jetson-tk1" ] ; then
			return
	fi

	if [ ! -w /sys/kernel/debug/tegra_fan/target_pwm ]; then
		echo "Can't access Fan!"
		return
	fi

	case "${ACTION}" in
		show)
			echo "Fan: speed=`cat /sys/kernel/debug/tegra_fan/target_pwm`"
			;;
		store)
			store "/sys/kernel/debug/tegra_fan/target_pwm"
			;;
		*)
			FAN_SPEED=255
			echo "${FAN_SPEED}" > /sys/kernel/debug/tegra_fan/target_pwm
			;;
	esac
}

do_clusterswitch()
{
	case "${ACTION}" in
		show)
			if [ -d "/sys/kernel/cluster" ]; then
				ACTIVE_CLUSTER=`cat /sys/kernel/cluster/active`
				echo "CPU Cluster Switching: Active Cluster ${ACTIVE_CLUSTER}"
			else
				echo "CPU Cluster Switching: Disabled"
			fi
			;;
		store)
			if [ -d "/sys/kernel/cluster" ]; then
				store "/sys/kernel/cluster/immediate"
				store "/sys/kernel/cluster/force"
				store "/sys/kernel/cluster/active"
			fi
			;;
		*)
			if [ -d "/sys/kernel/cluster" ]; then
				echo 1 > /sys/kernel/cluster/immediate
				echo 0 > /sys/kernel/cluster/force
				echo G > /sys/kernel/cluster/active
			fi
			;;
	esac
}

do_hotplug()
{
	case "${ACTION}" in
		show)
			echo "Online CPUs: `cat /sys/devices/system/cpu/online`"
			;;
		store)
			for file in /sys/devices/system/cpu/cpu[0-9]/online; do
				store "${file}"
			done
			;;
		*)
			if [ "${SOCFAMILY}" != "tegra186" ]; then
				for file in /sys/devices/system/cpu/cpu*/online; do
					if [ `cat $file` -eq 0 ]; then
						echo 1 > "${file}"
					fi
				done
			fi
	esac
}

do_cpu()
{
	FREQ_GOVERNOR="cpufreq/scaling_governor"
	CPU_MIN_FREQ="cpufreq/scaling_min_freq"
	CPU_MAX_FREQ="cpufreq/scaling_max_freq"
	CPU_CUR_FREQ="cpufreq/scaling_cur_freq"
	CPU_SET_SPEED="cpufreq/scaling_setspeed"
	INTERACTIVE_SETTINGS="/sys/devices/system/cpu/cpufreq/interactive"
	SCHEDUTIL_SETTINGS="/sys/devices/system/cpu/cpufreq/schedutil"

	case "${ACTION}" in
		show)
			for folder in /sys/devices/system/cpu/cpu[0-9]; do
				CPU=`basename ${folder}`
				if [ -e "${folder}/${FREQ_GOVERNOR}" ]; then
					echo "$CPU: Gonvernor=`cat ${folder}/${FREQ_GOVERNOR}`" \
						"MinFreq=`cat ${folder}/${CPU_MIN_FREQ}`" \
						"MaxFreq=`cat ${folder}/${CPU_MAX_FREQ}`" \
						"CurrentFreq=`cat ${folder}/${CPU_CUR_FREQ}`"
				fi
			done
			;;
		store)
			store "/sys/module/qos/parameters/enable"

			for file in \
				/sys/devices/system/cpu/cpu[0-9]/cpufreq/scaling_min_freq; do
				store "${file}"
			done

			if [ "${SOCFAMILY}" = "tegra186" ]; then
				store "/sys/kernel/debug/tegra_cpufreq/M_CLUSTER/cc3/enable"
				store "/sys/kernel/debug/tegra_cpufreq/B_CLUSTER/cc3/enable"
			fi
			;;
		*)
			echo 0 > /sys/module/qos/parameters/enable

			if [ "${SOCFAMILY}" = "tegra186" ]; then
				echo 0 > /sys/kernel/debug/tegra_cpufreq/M_CLUSTER/cc3/enable 2>/dev/null
				echo 0 > /sys/kernel/debug/tegra_cpufreq/B_CLUSTER/cc3/enable 2>/dev/null
			fi

			for folder in /sys/devices/system/cpu/cpu[0-9]; do
				cat "${folder}/${CPU_MAX_FREQ}" > "${folder}/${CPU_MIN_FREQ}" 2>/dev/null
			done
			;;
	esac
}

do_gpu()
{
	case "${SOCFAMILY}" in
		tegra186)
			GPU_MIN_FREQ="/sys/devices/17000000.gp10b/devfreq/17000000.gp10b/min_freq"
			GPU_MAX_FREQ="/sys/devices/17000000.gp10b/devfreq/17000000.gp10b/max_freq"
			GPU_CUR_FREQ="/sys/devices/17000000.gp10b/devfreq/17000000.gp10b/cur_freq"
			GPU_RAIL_GATE="/sys/devices/17000000.gp10b/railgate_enable"
			;;
		tegra210)
			GPU_MIN_FREQ="/sys/devices/57000000.gpu/devfreq/57000000.gpu/min_freq"
			GPU_MAX_FREQ="/sys/devices/57000000.gpu/devfreq/57000000.gpu/max_freq"
			GPU_CUR_FREQ="/sys/devices/57000000.gpu/devfreq/57000000.gpu/cur_freq"
			GPU_RAIL_GATE="/sys/devices/57000000.gpu/railgate_enable"
			;;
		*)
			echo "Error! unsupported SOC ${SOCFAMILY}"
			exit 1;
			;;
	esac

	case "${ACTION}" in
		show)
			echo "GPU MinFreq=`cat ${GPU_MIN_FREQ}`" \
				"MaxFreq=`cat ${GPU_MAX_FREQ}`" \
				"CurrentFreq=`cat ${GPU_CUR_FREQ}`"
			;;
		store)
			store "${GPU_MIN_FREQ}"
			store "${GPU_RAIL_GATE}"
			;;
		*)
			echo 0 > "${GPU_RAIL_GATE}"
			cat "${GPU_MAX_FREQ}" > "${GPU_MIN_FREQ}"
			ret=$?
			if [ ${ret} -ne 0 ]; then
				echo "Error: Failed to max GPU frequency!"
			fi
			;;
	esac
}

do_emc()
{
	case "${SOCFAMILY}" in
		tegra186)
			EMC_ISO_CAP="/sys/kernel/nvpmodel_emc_cap/emc_iso_cap"
			EMC_MIN_FREQ="/sys/kernel/debug/bpmp/debug/clk/emc/min_rate"
			EMC_MAX_FREQ="/sys/kernel/debug/bpmp/debug/clk/emc/max_rate"
			EMC_CUR_FREQ="/sys/kernel/debug/clk/emc/clk_rate"
			EMC_UPDATE_FREQ="/sys/kernel/debug/bpmp/debug/clk/emc/rate"
			EMC_FREQ_OVERRIDE="/sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked"
			;;
		tegra210)
			EMC_MIN_FREQ="/sys/kernel/debug/tegra_bwmgr/emc_min_rate"
			EMC_MAX_FREQ="/sys/kernel/debug/tegra_bwmgr/emc_max_rate"
			EMC_CUR_FREQ="/sys/kernel/debug/clk/override.emc/clk_rate"
			EMC_UPDATE_FREQ="/sys/kernel/debug/clk/override.emc/clk_update_rate"
			EMC_FREQ_OVERRIDE="/sys/kernel/debug/clk/override.emc/clk_state"
			;;
		*)
			echo "Error! unsupported SOC ${SOCFAMILY}"
			exit 1;
			;;

	esac
	case "${ACTION}" in
		show)
			echo "EMC MinFreq=`cat ${EMC_MIN_FREQ}`" \
				"MaxFreq=`cat ${EMC_MAX_FREQ}`" \
				"CurrentFreq=`cat ${EMC_CUR_FREQ}`" \
				"FreqOverride=`cat ${EMC_FREQ_OVERRIDE}`"
			;;
		store)
			store "${EMC_FREQ_OVERRIDE}"
			;;
		*)
			if [ "${SOCFAMILY}" = "tegra186" ]; then
				emc_cap=`cat "${EMC_ISO_CAP}"`
				if [ "$emc_cap" -eq 0 ]; then
					cat "${EMC_MAX_FREQ}" > "${EMC_UPDATE_FREQ}"
				else
					echo "$emc_cap" >  "${EMC_UPDATE_FREQ}"
				fi
			else
				cat "${EMC_MAX_FREQ}" > "${EMC_UPDATE_FREQ}"
			fi

			echo 1 > "${EMC_FREQ_OVERRIDE}"
			;;
	esac
}

main ()
{
	while [ -n "$1" ]; do
		case "$1" in
			--show)
				echo "SOC family:${SOCFAMILY}  Machine:${machine}"
				ACTION=show
				;;
			--store)
				[ -n "$2" ] && CONF_FILE=$2
				ACTION=store
				shift 1
				;;
			--restore)
				[ -n "$2" ] && CONF_FILE=$2
				ACTION=restore
				shift 1
				;;
			-h|--help)
				usage
				exit 0
				;;
			*)
				usage "Unknown option: $1"
				exit 1
				;;
		esac
		shift 1
	done

	[ `whoami` != root ] && \
		echo Error: Run this script\($0\) as a root user && exit 1

	case $ACTION in
		store)
			if [ -e "${CONF_FILE}" ]; then
				echo "File $CONF_FILE already exists. Can I overwrite it? Y/N:"
				read answer
				case $answer in
					y|Y)
						rm -f $CONF_FILE
						;;
					*)
						echo "Error: file $CONF_FILE already exists!"
						exit 1
						;;
				esac
			fi
			;;
		restore)
			if [ ! -e "${CONF_FILE}" ]; then
				echo "Error: $CONF_FILE file not found !"
				exit 1
			fi
			restore
			exit 0
			;;
	esac

	do_hotplug
	do_clusterswitch
	do_cpu
	do_gpu
	do_emc
	do_fan
}

if [ -e "/sys/devices/soc0/family" ]; then
	SOCFAMILY="`cat /sys/devices/soc0/family`"
	if [ -e "/sys/devices/soc0/machine" ]; then
		machine=`cat /sys/devices/soc0/machine`
	fi
elif [ -e "/proc/device-tree/compatible" ]; then
	grep "nvidia,tegra210" /proc/device-tree/compatible &>/dev/null
	if [ $? -eq 0 ]; then
		SOCFAMILY="tegra210"
	else
		grep "nvidia,tegra186" /proc/device-tree/compatible &>/dev/null
		if [ $? -eq 0 ]; then
			SOCFAMILY="tegra186"
		fi
	fi

	if [ -e "/proc/device-tree/model" ]; then
		machine="`cat /proc/device-tree/model`"
	fi
fi

main $@
exit 0
