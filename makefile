.PHONY: help
.DEFAULT_GOAL := help

install-mamba: ## install mamba (conda replacement)
	./install_mamba.sh

build-env: ## Setup environment (only when updating python or poetry version)
	conda-lock -f environment.yml -p osx-64 -p linux-64

create-ailedeau: ## Create copy of environment
	@if conda env list | grep -q ailedeau; then \
		conda env remove --name ailedeau; \
	else \
		echo "Environment 'ailedeau' does not exist, creating."; \
	fi
	${HOME}/conda/bin/conda-lock install --name ailedeau conda-lock.yml

install-poetry: ## Be sure to activate env
	poetry install

