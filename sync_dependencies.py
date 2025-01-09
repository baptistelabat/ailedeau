import toml

# Load the pyproject.toml file
with open("pyproject.toml", "r") as file:
    toml_data = toml.load(file)

# Extract dependencies from [tool.poetry.dependencies] and [tool.poetry.group.dev.dependencies]
dependencies = toml_data.get("tool", {}).get("poetry", {}).get("dependencies", {})
dev_dependencies = toml_data.get("tool", {}).get("poetry", {}).get("group", {}).get("dev", {}).get("dependencies", {})

# Format dependencies for the [project] section
project_dependencies = [
    f"{key}>={value[1:]}" if isinstance(value, str) else f"{key}>={value.get('version')[1:]}"
    for key, value in {**dependencies, **dev_dependencies}.items()
]

# Check if the [project] section already exists and update or create it
if "project" in toml_data:
    toml_data["project"]["dependencies"] = project_dependencies
else:
    toml_data["project"] = {"name": toml_data["tool"]["poetry"]["name"], "version": toml_data["tool"]["poetry"]["version"], "dependencies": project_dependencies}

# Write back to pyproject.toml
with open("pyproject.toml", "w") as file:
    toml.dump(toml_data, file)

print("Dependencies synced successfully!")
