#!/usr/bin/python3

import brahma.brahma_create as brahma_create
import brahma.BrahmaWorkspaceSettings as BrahmaWorkspaceSettings
import brahma.BrahmaWorkspacePaths as BrahmaWorkspacePaths
import argparse
import os

def test_brahma_create(tmp_path):
    settings = BrahmaWorkspaceSettings.BrahmaWorkspaceSettings()
    paths = BrahmaWorkspacePaths.BrahmaWorkspacePaths(tmp_path)
    args = argparse.Namespace()
    args.config = "default"
    args.force = False
    args.repository = []
    brahma_create.run(paths, settings, args)
    assert os.path.isdir(os.path.join(tmp_path, "source"))
