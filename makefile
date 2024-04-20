install-mamba: ## install mamba (conda replacement)
	./install_mamba.sh

build-env: ## Setup environment (only when updating python or poetry version)
	conda-lock -f environment.yml -p osx-64 -p linux-64

create-ailedeau: ## Create copy of environment
	mamba env remove --name ailedeu
	conda-lock install --name ailedeau conda-lock.yml

install-poetry: ## Be sure to activate env
	poetry install

