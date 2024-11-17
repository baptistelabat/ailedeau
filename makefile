PROJECT = ailedeau
install-mamba: ## install mamba (conda replacement)
	./install_mamba.sh

build-env: ## Setup environment (only when updating python or poetry version)
	conda-lock -f environment.yml -p osx-64 -p linux-64

create-ailedeau: ## Create copy of environment
	conda env remove --name $(PROJECT)
	conda-lock install --name $(PROJECT) conda-lock.yml

install-env:
	@if conda env list | grep -q $(PROJECT); then \
		conda env remove --name $(PROJECT); \
	else \
		echo "Environment $(PROJECT) does not exist, creating."; \
	fi
	${HOME}/conda/bin/conda-lock install --name $(PROJECT) conda-lock.yml

install-poetry: ## Be sure to activate env
	poetry install

