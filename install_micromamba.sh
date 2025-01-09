#!/bin/bash

"${SHELL}" <(curl -L micro.mamba.pm/install.sh -b -f)

micromamba install --channel=conda-forge --name=base conda-lock