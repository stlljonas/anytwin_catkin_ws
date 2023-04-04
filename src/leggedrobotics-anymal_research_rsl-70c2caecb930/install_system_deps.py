#!/usr/bin/env python

import argparse
import os
import sys
import subprocess
import textwrap
from catkin_pkg import packages as catkin_pkgs

try:
    import progressbar
except ImportError:
    sys.exit("Failed to import progressbar module. Please install it by running 'sudo apt install python-progressbar'")


# class inspired by https://github.com/mikeferguson/buildbot-ros
class RosdepResolver:
    def __init__(self, rosDistro):
        self.r2a = {}

        self.initDb(rosDistro)

    def initDb(self, rosDistro):
        process = subprocess.Popen(['rosdep', '--rosdistro='+rosDistro, 'db'],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = process.communicate()
        if process.returncode != 0:
            print(out)
            print(err)
            return False

        out = str(out.decode('utf8'))
        db = out.splitlines()

        print("Initialize package dependencies list...")

        for entry in db:
            split_entry = entry.split(' -> ')
            if len(split_entry) < 2:
                continue
            ros_entry = split_entry[0]
            apt_entries = split_entry[1].split(' ')
            self.r2a[ros_entry] = apt_entries

    def toApt(self, rosEntries):
        """ get corresponding apt packages from rosdep key
        :param rosEntries: List or string containing package name(s) to be resolved
        :return: Dict of rosdep key => set of resolved apt package names
        """
        if type(rosEntries) is str:
            rosEntries = [rosEntries]

        print("Scanning PPAs for dependencies...")

        res = {}
        for rosKey in rosEntries:
            if (not rosKey.endswith("-pip")) and rosKey in self.r2a:
                res.setdefault(rosKey, set())
                for aptEntry in self.r2a[rosKey]:
                    res[rosKey].add(aptEntry)
        return res

    def toPip(self, rosEntries):
        """ get corresponding pip packages from rosdep key
        :param rosEntries: List or string containing package name(s) to be resolved
        :return: Dict of rosdep key => set of resolved pip package names
        """
        if type(rosEntries) is str:
            rosEntries = [rosEntries]

        print("Scanning PIP for dependencies...")

        res = {}
        for rosKey in rosEntries:
            if rosKey.endswith("-pip") and rosKey in self.r2a:
                res.setdefault(rosKey, set())
                for pipEntry in self.r2a[rosKey]:
                    res[rosKey].add(pipEntry)
        return res


def listDepsRecursive(path, packages):
    """
    List all dependencies of given packages recursively
    :param path:        Path to the workspace the packages are located in
    :param packages:    List of package names to get list of recursive dependencies
    :return:            Dict (packageName => set of dependency names) and boolean indicating whether all packages have
                        been found.
    """
    catkinPkgs = catkin_pkgs.find_packages(path)

    upstreamDependencies = {}
    for pkg in catkinPkgs.values():
        upstreamDependencies.setdefault(pkg.name, set())
        allDeps = (pkg.build_depends + pkg.buildtool_depends + pkg.test_depends + pkg.doc_depends + pkg.exec_depends
                   + pkg.buildtool_export_depends + pkg.build_export_depends)
        upstreamDependencies[pkg.name].update([d.name for d in allDeps])

    upstreamDependenciesRecursive = {}
    for pkgName, deps in upstreamDependencies.items():
        # fill recursive upstream dependencies
        upstreamDependenciesRecursive.setdefault(pkgName, set())
        depsToFill = deps
        while depsToFill:
            upstreamDependenciesRecursive[pkgName].update(depsToFill)
            newDeps = set()
            for dep in depsToFill:
                if dep in upstreamDependencies:
                    newDeps.update(upstreamDependencies[dep])
            depsToFill = newDeps

    foundAll = True
    if packages:
        for name in packages:
            if name not in upstreamDependencies:
                print("Package {} not found".format(name))
                foundAll = False

    return upstreamDependenciesRecursive, foundAll


def installDepsRecursive(path, packages, rosdistro, install_ws_packages):
    """
    Install all dependencies of given packages recursively
    :param path:                Path to the workspace the packages are located in
    :param packages:            List of package names to get list of recursive dependencies
    :param rosdistro:           ROS Distribution to use
    :return:                    True on success
    """
    upstreamDependencies, foundAll = listDepsRecursive(path, packages)
    if not foundAll:
        return False

    if packages:
        allDeps = set([dep for name in packages for dep in upstreamDependencies[name]
                       if dep not in upstreamDependencies or install_ws_packages])
    else:
        allDeps = set([dep for deps in upstreamDependencies.values() for dep in deps
                       if dep not in upstreamDependencies or install_ws_packages])

    if not allDeps:
        # set is empty, nothing to do
        return True

    rosdep = RosdepResolver(rosdistro)

    # text wrapper, formats package lists to fit regular screens without cutting text. Introduces indentation.
    text_wrapper = textwrap.TextWrapper(initial_indent='   ', subsequent_indent='   ', width = 150, break_long_words=False, break_on_hyphens=False)

    # installation by PPA.
    aptInstalls = rosdep.toApt(allDeps)
    aptCmd = ['sudo', 'apt', 'install', '--no-remove', '--yes']
    if aptInstalls:
        # extract list of packages.
        allDeps = allDeps.difference(aptInstalls.keys())
        pkgs = [pkg for pkgs in aptInstalls.values() for pkg in pkgs]

        # print list of packages.
        print('Installing the following packages from PPA:')
        for line in text_wrapper.wrap(text='{}'.format(' '.join(sorted(pkgs)))):
            print(line)

        process = subprocess.Popen(aptCmd + pkgs,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = process.communicate()
        if process.returncode != 0:
            print(out)
            print(err)
            return False

        # try to install -dev versions. This is a workaround for packages which are released by our pipeline as well as
        # the official ros, e.g. grid_map.
        pkgs_dev = ["{}-dev".format(pkg) for pkgs in aptInstalls.values() for pkg in pkgs
                    if pkg.startswith('ros-melodic-')]

        # Progress bar init.
        progress_bar = progressbar.ProgressBar(maxval=len(pkgs_dev), \
            widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])
        progress_bar.start()
        pkg_counter = 0

        # need to try one-by-one for the time being, otherwise apt will return with error immediately if it cannot
        # find one of the supplied packages
        for pkg in pkgs_dev:
            process = subprocess.Popen(aptCmd + [pkg], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            process.communicate()

            pkg_counter += 1
            progress_bar.update(pkg_counter)
        progress_bar.finish()

        print('Package installation from PPA finished.')

    # installation by pip.
    pipInstalls = rosdep.toPip(allDeps)
    if pipInstalls:
        # extract list of packages.
        allDeps = allDeps.difference(pipInstalls.keys())
        pkgs = [pkg for pkgs in pipInstalls.values() for pkg in pkgs]

        # Print list of packages.
        print('Installing the following packages from pip:')
        for line in text_wrapper.wrap(text='{}'.format(' '.join(sorted(pkgs)))):
            print(line)

        process = subprocess.Popen(['sudo', 'pip', 'install', '-U'] + pkgs,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = process.communicate()
        if process.returncode != 0:
            print(out)
            print(err)
            return False

        print('Package installation from pip finished.')

    # remaining items are not registered rosdep keys, try to install them directly.
    success = True
    if allDeps:
        # keep track of unresolved packages.
        unresolved_pkgs = []

        # progress bar init.
        progress_bar = progressbar.ProgressBar(maxval=len(allDeps), \
            widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])
        progress_bar.start()
        pkg_counter = 0

        # print list of packages.
        print('Trying to resolve the following packages directly:')
        for line in text_wrapper.wrap(text='{}'.format(' '.join(allDeps))):
            print(line)

        for dep in allDeps:
            pkg_name = dep.replace('_', '-')
            pkg_found = False

            # try to install non-dev pkg
            process = subprocess.Popen(aptCmd + [pkg_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            process.communicate()
            if process.returncode == 0:
                pkg_found = True

            # also try to install '-dev' package
            process = subprocess.Popen(aptCmd + ["{}-dev".format(pkg_name)], stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE)
            process.communicate()
            if process.returncode == 0:
                pkg_found = True

            # if package was not found in apt, try with ros prefix
            if not pkg_found:
                pkg_name = 'ros-{}-{}'.format(rosdistro, dep.replace('_', '-'))
                process = subprocess.Popen(aptCmd + [pkg_name],
                                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                process.communicate()
                if process.returncode == 0:
                    pkg_found = True

                # also try to install '-dev' package
                process = subprocess.Popen(aptCmd + ["{}-dev".format(pkg_name)], stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE)
                process.communicate()
                if process.returncode == 0:
                    pkg_found = True

                if not pkg_found:
                    unresolved_pkgs.append(dep)
                    success = False

            if pkg_found:
                pkg_counter += 1
                progress_bar.update(pkg_counter)
        progress_bar.finish()

        if unresolved_pkgs:
            print('The following {} dependencies could NOT be resolved directly:'.format(len(unresolved_pkgs)))
            for line in text_wrapper.wrap(text='{}'.format(' '.join(sorted(unresolved_pkgs)))):
                print(line)
        else:
            print('Package installation by direct resolution finished.')

    return success


def main():
    parser = argparse.ArgumentParser(description="Install (or list) dependencies of given packages recursively.")
    parser.add_argument(dest="packages", type=str, nargs="*", help="List of packages to install dependencies of. "
                                                                   "Leave empty to install deps of all packages found "
                                                                   "in specified path.")
    parser.add_argument('--path', type=str, default=os.getcwd(),
                        help="Path to the catkin source workspace (Default: %(default)s).")
    parser.add_argument('--dry-run', action="store_true", help="Do not install dependencies, only print list.")
    parser.add_argument('--rosdistro', type=str, default=os.environ.get('ROS_DISTRO', ''),
                        help="ROS distribution to use (Default: %(default)s)")
    parser.add_argument('--install-ws-packages', action="store_true",
                        help="Whether to install debian packages of packages which have been found in the workspace. "
                             "If this flag is used, a package has to be specified.")

    args = parser.parse_args()

    if args.dry_run:
        upstreamDependencies, foundAll = listDepsRecursive(args.path, args.packages)
        for name in args.packages:
            print("{} depends on: {}".format(name, ' '.join(upstreamDependencies[name])))

        if not foundAll:
            sys.exit(-1)

    else:
        if not args.rosdistro:
            sys.exit("Unable to get ros distribution from env, please specify with --rosdistro")

        # prevent to run with install_ws_packages and without specified packages, this will install things which
        # should usually not be installed (e.g. anymal_pc package)
        if args.install_ws_packages and not args.packages:
            sys.exit("Cannot run with --install-ws-packages flag without a specified package.")

        if not installDepsRecursive(args.path, args.packages, args.rosdistro, args.install_ws_packages):
            sys.exit("Failed to install all dependencies")
        else:
            print('All dependencies installed successfully!')


if __name__ == "__main__":
    main()
