#!/usr/bin/env python3

from pathlib import Path

import regex as re


DATA = {
    "authors": {
        "Vaibhav Gupta": "vaibhav.gupta@epfl.ch",
    },
    "maintainers": {
        "Saurav Aryan": "saurav.aryan@epfl.ch"
    },
    "owner": "Learning Algorithms and Systems Laboratory, EPFL, Switzerland",
    "owner_website": "lasa.epfl.ch",
    "project_name": "allegro_hand_gazebo",
    "year": 2021,
}

LICENSE_TEMPLATE = """Copyright (C) {year:d} {owner:s}
Authors: {authors_maintainers:s}
Website: {owner_website:s}

This file is part of `{project_name:s}`.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>."""

TYPE_SETTINGS = {
    "script": {
        "extensions": [".sh"],
        "exclude_filenames": [],
        "keepFirst": re.compile(r'^#!|^# -\*-'),
        "blockCommentStartPattern": None,
        "blockCommentEndPattern": None,
        "lineCommentStartPattern": re.compile(r'^\s*#'),
        "lineCommentEndPattern": None,
        "headerStartLine": "##",
        "headerEndLine": "##",
        "headerLinePrefix": "## ",
        "headerLineSuffix": None
    },
    "python": {
        "extensions": [".py"],
        "exclude_filenames": ["add_license.py"],
        "keepFirst": re.compile(r'^#!|^# +pylint|^# +-\*-|^# +coding|^# +encoding|^# +type|^# +flake8'),
        "blockCommentStartPattern": None,
        "blockCommentEndPattern": None,
        "lineCommentStartPattern": re.compile(r'^\s*#'),
        "lineCommentEndPattern": None,
        "headerStartLine": "#",
        "headerEndLine": "#",
        "headerLinePrefix": "# ",
        "headerLineSuffix": None
    },
    "xml": {
        "extensions": [".xml", ".launch", ".urdf", ".xacro"],
        "exclude_filenames": ["package.xml"],
        "keepFirst": re.compile(r'^\s*<\?xml.*\?>'),
        "blockCommentStartPattern": re.compile(r'^\s*<!--'),
        "blockCommentEndPattern": re.compile(r'-->\s*$'),
        "lineCommentStartPattern": None,
        "lineCommentEndPattern": None,
        "headerStartLine": "<!--",
        "headerEndLine": "  -->",
        "headerLinePrefix": "\t",
        "headerLineSuffix": None
    },
    "c": {
        "extensions": [".c", ".cc", ".cpp", "c++", ".h", ".hpp"],
        "exclude_filenames": [],
        "keepFirst": None,
        "blockCommentStartPattern": re.compile(r'^\s*/\*'),
        "blockCommentEndPattern": re.compile(r'\*/\s*$'),
        "lineCommentStartPattern": re.compile(r'^\s*//'),
        "lineCommentEndPattern": None,
        "headerStartLine": "/*",
        "headerEndLine": " */",
        "headerLinePrefix": " * ",
        "headerLineSuffix": None
    },
    "csharp": {
        "extensions": [".cs"],
        "exclude_filenames": [],
        "keepFirst": None,
        "blockCommentStartPattern": None,
        "blockCommentEndPattern": None,
        "lineCommentStartPattern": re.compile(r'^\s*//'),
        "lineCommentEndPattern": None,
        "headerStartLine": None,
        "headerEndLine": None,
        "headerLinePrefix": "// ",
        "headerLineSuffix": None
    },
    "docker": {
        "extensions": [".dockerfile"],
        "filenames": ["Dockerfile"],
        "exclude_filenames": [],
        "keepFirst": None,
        "blockCommentStartPattern": None,
        "blockCommentEndPattern": None,
        "lineCommentStartPattern": re.compile(r'^\s*#'),
        "lineCommentEndPattern": None,
        "headerStartLine": "##",
        "headerEndLine": "##",
        "headerLinePrefix": "## ",
        "headerLineSuffix": None
    },
    "yaml": {
        "extensions": [".yaml", ".yml"],
        "exclude_filenames": [],
        "keepFirst": None,
        "blockCommentStartPattern": None,
        "blockCommentEndPattern": None,
        "lineCommentStartPattern": re.compile(r'^\s*#'),
        "lineCommentEndPattern": None,
        "headerStartLine": "##",
        "headerEndLine": "##",
        "headerLinePrefix": "## ",
        "headerLineSuffix": None
    },
    "matlab": {
        "extensions": [".m"],
        "exclude_filenames": [],
        "keepFirst": None,
        "blockCommentStartPattern": None,
        "blockCommentEndPattern": None,
        "lineCommentStartPattern": re.compile(r'^\s*%'),
        "lineCommentEndPattern": None,
        "headerStartLine": "%",
        "headerEndLine": "%",
        "headerLinePrefix": "% ",
        "headerLineSuffix": None
    },
}


def get_license(license, settings):
    """Modify license for file type

    Args:
        license (str): License to be inserted
        settings (Dict): File type settings

    Returns:
        str: Modified license to be inserted in the file
    """
    lines = []
    header_start_line = settings["headerStartLine"]
    header_end_line = settings["headerEndLine"]
    header_line_prefix = settings["headerLinePrefix"]
    header_line_suffix = settings["headerLineSuffix"]
    if header_start_line is not None:
        lines.append(header_start_line.rstrip())
    for line in license.split("\n"):
        tmp = line
        if header_line_prefix is not None:
            tmp = header_line_prefix + tmp
        if header_line_suffix is not None:
            tmp = tmp + header_line_suffix
        lines.append(tmp.rstrip())
    if header_end_line is not None:
        lines.append(header_end_line.rstrip())
    return "\n".join(lines) + "\n"


def update_license(data):
    """Add license information to the template

    Args:
        data (Dict): Data to be updated in the template

    Returns:
        str: Updated license
    """
    authors = [
        "{:s} ({:s})".format(name, DATA["authors"][name])
        for name in DATA["authors"].keys()
    ]
    maintainers = [
        "{:s} ({:s}) [Maintainer]".format(name, DATA["maintainers"][name])
        for name in DATA["maintainers"].keys()
    ]
    authors_maintainers = "\n\t" + "\n\t".join([*authors, *maintainers])
    license = LICENSE_TEMPLATE.format(
        year=data["year"],
        owner=data["owner"],
        owner_website=data["owner_website"],
        project_name=data["project_name"],
        authors_maintainers=authors_maintainers,
    )
    return license


def make_dirlist(folder, exclude_folders, fnpatterns, exclude_fnpatterns):
    """List of all files matching fnpatterns and not matching exclude_fnpatterns

    Args:
        folder (str): Directory to look into
        exclude_folders (List[str]): Directories to skip
        fnpatterns (List[str]): Patterns to match
        exclude_fnpatterns (List[str]): Patterns to exclude

    Returns:
        List[PosixPath]: List of all matching files
    """
    matches = []
    for fnpattern in fnpatterns:
        matches.extend(Path(folder).rglob(fnpattern))
    matches = set(matches)
    for exclude_fnpattern in exclude_fnpatterns:
        matches = matches - set(Path(folder).rglob(exclude_fnpattern))
    return [
        path for path in matches
        if not any([
            Path(exclude_folder) in path.parents
            for exclude_folder in exclude_folders
        ])
    ]


def parse_file(filepath, settings):  # noqa: C901 (Skip complexity check)
    """Parse file for lines to skip, header, and available license

    Args:
        filepath (PosixPath): Path to the file
        settings (Dict): Settings Dictionary

    Returns:
        Dict: a dictionary with the following entries
            - skip: number of lines at the beginning to skip (always keep them when replacing or adding something)
                can also be seen as the index of the first line not to skip
            - headStart: index of first line of detected header, or None if no header detected
            - headEnd: index of last line of detected header, or None
            - haveLicense: found a line that matches a pattern that indicates this could be a license header
    """
    skip = 0
    head_start = None
    head_end = None
    have_license = False
    isBlockHeader = False

    with filepath.open('r') as f:
        lines = f.readlines()

    emptyPattern = re.compile(r'^\s*$')
    licensePattern = re.compile(r"license", re.IGNORECASE)

    keep_first = settings["keepFirst"]
    block_comment_start_pattern = settings["blockCommentStartPattern"]
    block_comment_end_pattern = settings["blockCommentEndPattern"]
    line_comment_start_pattern = settings["lineCommentStartPattern"]
    i = 0
    for line in lines:
        if (i == 0) and keep_first and keep_first.findall(line):
            skip = i + 1
        elif emptyPattern.findall(line):
            pass
        elif block_comment_start_pattern and block_comment_start_pattern.findall(line):
            head_start = i
            isBlockHeader = True
            break
        elif line_comment_start_pattern and line_comment_start_pattern.findall(line):
            head_start = i
            break
        elif not block_comment_start_pattern and \
                line_comment_start_pattern and \
                line_comment_start_pattern.findall(line):
            head_start = i
            break
        else:
            # we have reached something else, so no header in this file
            return {
                "skip": skip,
                "lines": lines,
                "headStart": None,
                "headEnd": None,
                "haveLicense": have_license
            }
        i = i + 1

    # now we have either reached the end, or we are at a line where a block start or line comment occurred
    # if we have reached the end, return default dictionary without info
    if i == len(lines):
        return {
            "skip": skip,
            "lines": lines,
            "headStart": head_start,
            "headEnd": head_end,
            "haveLicense": have_license
        }

    # otherwise process the comment block until it ends
    if isBlockHeader:
        for j in range(i, len(lines)):
            if licensePattern.findall(lines[j]):
                have_license = True
            elif block_comment_end_pattern.findall(lines[j]):
                return {
                    "skip": skip,
                    "lines": lines,
                    "headStart": head_start,
                    "headEnd": j,
                    "haveLicense": have_license
                }
        # if we went through all the lines without finding an end, maybe we have some syntax error or some other
        # unusual situation, so lets return no header
        return {
            "skip": skip,
            "lines": lines,
            "headStart": None,
            "headEnd": None,
            "haveLicense": have_license
        }
    else:
        # Just comment lines
        for j in range(i, len(lines)):
            if line_comment_start_pattern.findall(lines[j]) and licensePattern.findall(lines[j]):
                have_license = True
            elif not line_comment_start_pattern.findall(lines[j]):
                return {
                    "skip": skip,
                    "lines": lines,
                    "headStart": i,
                    "headEnd": j - 1,
                    "haveLicense": have_license
                }
        # if we went through all the lines without finding the end of the block, it could be that the whole
        # file only consisted of the header, so lets return the last line index
        return {
            "skip": skip,
            "lines": lines,
            "headStart": i,
            "headEnd": len(lines) - 1,
            "haveLicense": have_license
        }


def main(directory, exclude_directories=[]):
    """Main function

    Args:
        directory ([type]): Directory to traverse
        exclude_directories (List[str], optional): Directories to exclude. Defaults to [].
    """
    license = update_license(DATA)

    for t in TYPE_SETTINGS.keys():
        print("Current File Type --> {:s}".format(t))
        fnpatterns = []
        exclude_fnpatterns = []

        settings = TYPE_SETTINGS[t]
        exts = settings["extensions"]
        [fnpatterns.append("*" + ext) for ext in exts]
        if "filenames" in settings:
            fnpatterns.extend(settings["filenames"])
        if "exclude_filenames" in settings:
            exclude_fnpatterns.extend(settings["exclude_filenames"])
        files = make_dirlist(directory, exclude_directories, fnpatterns, exclude_fnpatterns)
        typed_license = get_license(license, settings)
        for filepath in files:
            print("\t", filepath)
            file_info = parse_file(filepath, settings)
            lines = file_info["lines"]
            skip = file_info["skip"]
            head_start = file_info["headStart"]
            head_end = file_info["headEnd"]
            have_license = file_info["haveLicense"]
            with filepath.open('w') as fw:
                # if we found a header, replace it otherwise, add it after the lines to skip
                if head_start is not None and head_end is not None and have_license:
                    # ----- Replace Header -----
                    # first write the lines before the header
                    fw.writelines(lines[0:head_start])
                    #  now write the new header from the template lines
                    fw.writelines(typed_license)
                    #  now write the rest of the lines
                    fw.writelines(lines[head_end + 1:])
                else:
                    # ----- Add Header -----
                    fw.writelines(lines[0:skip])
                    fw.writelines(typed_license)
                    if head_start is not None and not have_license:
                        # There is some header, but not license - add an empty line
                        fw.write("\n")
                    fw.writelines(lines[skip:])


if __name__ == '__main__':
    main(".", [".git"])
