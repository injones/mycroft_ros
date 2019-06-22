#!/usr/bin/env bash
#
# Copyright 2017 Mycroft AI Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
##########################################################################

# Set a default locale to handle output from commands reliably
export LANG=C

# exit on any error
set -Ee

cd $(dirname $0)
TOP=$( pwd -L )

# Parse the command line
opt_forcemimicbuild=false
opt_allowroot=false
opt_skipmimicbuild=false
opt_python=python3
param=""

if [ $(id -u) -eq 0 ] && [ "${opt_allowroot}" != true ] ; then
    echo "This script should not be run as root or with sudo."
    echo "If you really need to for this, rerun with --allow-root"
    exit 1
fi


function found_exe() {
    hash "$1" 2>/dev/null
}


if found_exe sudo ; then
    SUDO=sudo
elif [ "${opt_allowroot}" != true ]; then
    echo "This script requires \"sudo\" to install system packages. Please install it, then re-run this script."
    exit 1
fi


function get_YN() {
    # Loop until the user hits the Y or the N key
    echo -e -n "Choice [${CYAN}Y${RESET}/${CYAN}N${RESET}]: "
    while true; do
        read -N1 -s key
        case $key in
        [Yy])
            return 0
            ;;
        [Nn])
            return 1
            ;;
        esac
    done
}

if found_exe tput ; then
    GREEN="$(tput setaf 2)"
    BLUE="$(tput setaf 4)"
    CYAN="$(tput setaf 6)"
    YELLOW="$(tput setaf 3)"
    RESET="$(tput sgr0)"
    HIGHLIGHT=${YELLOW}
fi

echo "The standard location for Mycroft skills is under /opt/mycroft/skills."
    if [[ ! -d /opt/mycroft/skills ]] ; then
        echo "This script will create that folder for you.  This requires sudo"
        echo "permission and might ask you for a password..."
        setup_user=$USER
        setup_group=$( id -gn $USER )
        $SUDO mkdir -p /opt/mycroft/skills
        $SUDO chown -R ${setup_user}:${setup_group} /opt/mycroft
        echo "Created!"
    fi
    if [[ ! -d skills ]] ; then
        ln -s /opt/mycroft/skills skills
        echo "For convenience, a soft link has been created called 'skills' which leads"
        echo "to /opt/mycroft/skills."
    fi


function os_is() {
    [[ $(grep "^ID=" /etc/os-release | awk -F'=' '/^ID/ {print $2}' | sed 's/\"//g') == $1 ]]
}

function redhat_common_install() {
    $SUDO yum install -y cmake gcc-c++ git python34 python34-devel libtool libffi-devel openssl-devel autoconf automake bison swig portaudio-devel mpg123 flac curl libicu-devel python34-pkgconfig libjpeg-devel fann-devel python34-libs pulseaudio
    git clone https://github.com/libfann/fann.git
    cd fann
    git checkout b211dc3db3a6a2540a34fbe8995bf2df63fc9939
    cmake .
    $SUDO make install
    cd "${TOP}"
    rm -rf fann

}

function install_deps() {
    echo "Installing packages..."
    if found_exe zypper ; then
        # OpenSUSE
        $SUDO zypper install -y git python3 python3-devel libtool libffi-devel libopenssl-devel autoconf automake bison swig portaudio-devel mpg123 flac curl libicu-devel pkg-config libjpeg-devel libfann-devel python3-curses pulseaudio
        $SUDO zypper install -y -t pattern devel_C_C++
    elif found_exe yum && os_is centos ; then
        # CentOS
        $SUDO yum install epel-release
        redhat_common_install
    elif found_exe yum && os_is rhel ; then
        # Redhat Enterprise Linux
        $SUDO yum install -y wget
        wget https://dl.fedoraproject.org/pub/epel/epel-release-latest-7.noarch.rpm
        $SUDO yum install -y epel-release-latest-7.noarch.rpm
        rm epel-release-latest-7.noarch.rpm
        redhat_common_install
    elif found_exe apt-get ; then
        # Debian / Ubuntu
        $SUDO apt-get install -y git python3 python3-dev python-setuptools python-gobject-2-dev libtool libffi-dev libssl-dev autoconf automake bison swig libglib2.0-dev portaudio19-dev mpg123 screen flac curl libicu-dev pkg-config automake libjpeg-dev libfann-dev build-essential jq
    elif found_exe pacman; then
        # Arch Linux
        $SUDO pacman -S --needed --noconfirm git python python-pip python-setuptools python-virtualenv python-gobject python-virtualenvwrapper libffi swig portaudio mpg123 screen flac curl icu libjpeg-turbo base-devel jq pulseaudio pulseaudio-alsa
        pacman -Qs "^fann$" &> /dev/null || (
            git clone  https://aur.archlinux.org/fann.git
            cd fann
            makepkg -srciA --noconfirm
            cd ..
            rm -rf fann
        )
    elif found_exe dnf ; then
        # Fedora
        $SUDO dnf install -y git python3 python3-devel python3-pip python3-setuptools python3-virtualenv pygobject3-devel libtool libffi-devel openssl-devel autoconf bison swig glib2-devel portaudio-devel mpg123 mpg123-plugins-pulseaudio screen curl pkgconfig libicu-devel automake libjpeg-turbo-devel fann-devel gcc-c++ redhat-rpm-config jq
    else
    	echo
        echo "${GREEN}Could not find package manager"
        echo "${GREEN}Make sure to manually install:${BLUE} git python 2 python-setuptools python-virtualenv pygobject virtualenvwrapper libtool libffi openssl autoconf bison swig glib2.0 portaudio19 mpg123 flac curl fann g++"
        echo ${RESET}
    fi
}

install_deps

# Check whether to build mimic (it takes a really long time!)
build_mimic="n"
if [[ ${opt_forcemimicbuild} == true ]] ; then
    build_mimic="y"
else
    # first, look for a build of mimic in the folder
    has_mimic=""
    if [[ -f ${TOP}/mimic/bin/mimic ]] ; then
        has_mimic=$( ${TOP}/mimic/bin/mimic -lv | grep Voice ) || true
    fi

    # in not, check the system path
    if [ "$has_mimic" == "" ] ; then
        if [ -x "$(command -v mimic)" ] ; then
            has_mimic="$( mimic -lv | grep Voice )" || true
        fi
    fi

    if [ "$has_mimic" == "" ]; then
        if [[ ${opt_skipmimicbuild} == true ]] ; then
            build_mimic="n"
        else
            build_mimic="y"
        fi
    fi
fi

build_mimic="n"
SYSMEM=$( free | awk '/^Mem:/ { print $2 }' )
MAXCORES=$(($SYSMEM / 512000))
MINCORES=1
CORES=$( nproc )

# ensure MAXCORES is > 0
if [[ ${MAXCORES} -lt 1 ]] ; then
    MAXCORES=${MINCORES}
fi

# look for positive integer
if ! [[ ${CORES} =~ ^[0-9]+$ ]] ; then
    CORES=${MINCORES}
elif [[ ${MAXCORES} -lt ${CORES} ]] ; then
    CORES=${MAXCORES}
fi

echo "Building with $CORES cores."

#build and install pocketsphinx
#cd ${TOP}
#${TOP}/scripts/install-pocketsphinx.sh -q
#build and install mimic
cd "${TOP}"

if [[ "$build_mimic" == "y" ]] || [[ "$build_mimic" == "Y" ]] ; then
    echo "WARNING: The following can take a long time to run!"
    "${TOP}/scripts/install-mimic.sh" " ${CORES}"
else
    echo "Skipping mimic build."
fi

# create and set permissions for logging
if [[ ! -w /var/log/mycroft/ ]] ; then
    # Creating and setting permissions
    echo "Creating /var/log/mycroft/ directory"
    if [[ ! -d /var/log/mycroft/ ]] ; then
        $SUDO mkdir /var/log/mycroft/
    fi
    $SUDO chmod 777 /var/log/mycroft/
fi
