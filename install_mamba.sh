#!/bin/bash
# pseudocode to check if mamba is installed
#if conda list

# https://github.com/conda-forge/miniforge
if conda info; then
  echo " --- conda already installed"
else
  echo " --- installing conda"
  wget -O Miniforge3.sh "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
  bash Miniforge3.sh -b -p "${HOME}/conda"
  rm Miniforge3.sh
fi

source "${HOME}/conda/etc/profile.d/conda.sh"
# For mamba support also run the following command
source "${HOME}/conda/etc/profile.d/mamba.sh"

mamba install --channel=conda-forge --name=base conda-lock