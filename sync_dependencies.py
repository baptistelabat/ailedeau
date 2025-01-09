import toml
import os
import shutil

# Load the pyproject.toml file
with open("pyproject.toml", "r") as file:
    toml_data = toml.load(file)

# Extract the name and version from the [tool.poetry] section
poetry_data = toml_data.get("tool", {}).get("poetry", {})
project_name = poetry_data.get("name", "ailedeau")  # Default to "ailedeau" if not found
project_version = poetry_data.get("version", "0.1.0")  # Default to "0.1.0" if not found

# Extract dependencies from [tool.poetry.dependencies] and [tool.poetry.group.dev.dependencies]
dependencies = poetry_data.get("dependencies", {})
dev_dependencies = poetry_data.get("group", {}).get("dev", {}).get("dependencies", {})

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

    # Find the [project] section (if it exists)
    project_start = None
    project_end = None
    for i, line in enumerate(lines):
        new_lines.append(line)
        if line.strip() == "[project]":
            project_start = i
            # Find the end of the [project] section
            for j in range(i + 1, len(lines)):
                if lines[j].strip() == "":
                    project_end = j
                    break
            if project_end is None:
                project_end = len(lines)

            break

    # If [project] section exists, modify it
    if project_start is not None:
        # Ensure name and version are included in the [project] section
        name_found = False
        version_found = False

        # Check for name and version
        for i in range(project_start, project_end):
            if lines[i].strip().startswith('name ='):
                name_found = True
            if lines[i].strip().startswith('version ='):
                version_found = True

        # Modify the section
        if not name_found:
            new_lines.insert(project_end, f'name = "{project_name}"\n')
        if not version_found:
            new_lines.insert(project_end, f'version = "{project_version}"\n')

        # # Remove the old dependencies if they exist and add the formatted ones
        # dep_start = None
        # dep_end = None
        # for i in range(project_start, project_end):
        #     if lines[i].strip().startswith("dependencies"):
        #         dep_start = i
        #         for j in range(i + 1, project_end):
        #             if lines[j].strip() == "":
        #                 dep_end = j
        #                 break
        #         break
        #
        # if dep_start is not None:
        #     # Remove old dependencies
        #     new_lines = new_lines[:dep_start]

        new_lines.append("dependencies = [\n")
        for i, dep in enumerate(formatted_dependencies):
            if i != len(formatted_dependencies) - 1:  # Don't add a comma after the last element
                new_lines.append(f"  {dep},\n")
            else:
                new_lines.append(f"  {dep}\n")
        new_lines.append("]\n")

        # Append any remaining content after the [project] section
        new_lines.extend(lines[project_end:])

    else:
        # If there's no [project] section, add it at the top of the file
        new_lines.append("[project]\n")
        new_lines.append(f'name = "{project_name}"\n')
        new_lines.append(f'version = "{project_version}"\n')
        new_lines.append("dependencies = [\n")
        for i, dep in enumerate(formatted_dependencies):
            if i != len(formatted_dependencies) - 1:  # Don't add a comma after the last element
                new_lines.append(f"  {dep},\n")
            else:
                new_lines.append(f"  {dep}\n")
        new_lines.append("]\n")
        # Append any remaining content
        new_lines.extend(lines)

# Write the new content to a temporary file
temp_file = "pyproject.toml.tmp"
with open(temp_file, "w") as temp_f:
    temp_f.writelines(new_lines)

# Replace the original file with the new file
os.remove("pyproject.toml")
shutil.move(temp_file, "pyproject.toml")

print("Dependencies synced successfully with proper formatting!")
