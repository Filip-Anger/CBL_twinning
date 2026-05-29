# Copyright 2024 falinux
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from ament_pep257.main import main
import pytest


@pytest.mark.pep257
@pytest.mark.linter
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
