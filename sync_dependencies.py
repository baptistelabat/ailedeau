import toml
import os
import shutil

# Load the pyproject.toml file
with open("pyproject.toml", "r") as file:
    toml_data = toml.load(file)

# Extract dependencies from [tool.poetry.dependencies] and [tool.poetry.group.dev.dependencies]
dependencies = toml_data.get("tool", {}).get("poetry", {}).get("dependencies", {})
dev_dependencies = toml_data.get("tool", {}).get("poetry", {}).get("group", {}).get("dev", {}).get("dependencies", {})

# Combine dependencies from both sections
combined_dependencies = {**dependencies, **dev_dependencies}

# Create a list of formatted dependencies
formatted_dependencies = [
    f'"{key}>= {value[1:]}"' if isinstance(value, str) else f'"{key}>= {value.get("version")[1:]}"'
    for key, value in combined_dependencies.items()
]

# Remove any empty or invalid dependencies (e.g. if there's an empty string)
formatted_dependencies = [dep for dep in formatted_dependencies if dep.strip()]

# Prepare new content for the pyproject.toml file
new_lines = []
with open("pyproject.toml", "r") as file:
    lines = file.readlines()

    # Check if the [project] section exists, if not, add it
    project_start = None
    for i, line in enumerate(lines):
        new_lines.append(line)
        if line.strip() == "[project]":
            project_start = i
            break

    # If [project] section exists, find and replace the dependencies
    if project_start is not None:
        dep_start = None
        dep_end = None

        for i in range(project_start, len(lines)):
            if "dependencies" in lines[i]:
                dep_start = i
                break

        if dep_start is not None:
            for i in range(dep_start, len(lines)):
                if lines[i].strip() == "]":
                    dep_end = i + 1
                    break

            # Append the new dependencies after the [project] section
            new_lines.append("dependencies = [\n")
            for i, dep in enumerate(formatted_dependencies):
                if i != len(formatted_dependencies) - 1:  # Don't add a comma after the last element
                    new_lines.append(f"  {dep},\n")
                else:
                    new_lines.append(f"  {dep}\n")
            new_lines.append("]\n")

            # Skip the old dependencies
            new_lines = new_lines[:dep_start]

    else:
        # If there's no [project] section, add it
        new_lines.append("[project]\n")
        new_lines.append('name = "ailedeau"\n')
        new_lines.append('version = "0.1.0"\n')
        new_lines.append("dependencies = [\n")
        for i, dep in enumerate(formatted_dependencies):
            if i != len(formatted_dependencies) - 1:  # Don't add a comma after the last element
                new_lines.append(f"  {dep},\n")
            else:
                new_lines.append(f"  {dep}\n")
        new_lines.append("]\n")

    # Keep any remaining content after the [project] section
    for line in lines[project_start + 1:] if project_start is not None else lines:
        new_lines.append(line)

# Write the new content to a temporary file
temp_file = "pyproject.toml.tmp"
with open(temp_file, "w") as temp_f:
    temp_f.writelines(new_lines)

# Replace the original file with the new file
os.remove("pyproject.toml")
shutil.move(temp_file, "pyproject.toml")

print("Dependencies synced successfully with proper formatting!")
