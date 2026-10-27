#!/usr/bin/env python3
"""Run Docker directly, or activate an existing docker group for this command."""
import grp
import os
import pwd
import shlex
import shutil
import sys

if len(sys.argv) < 2:
    raise SystemExit('usage: pc_docker.py DOCKER_ARGUMENT ...')
docker = shutil.which('docker')
if docker is None:
    raise SystemExit('Docker CLI not found on PATH')
command = [docker, *sys.argv[1:]]
socket_path = '/var/run/docker.sock'
use_local_socket = not (os.environ.get('DOCKER_HOST') or os.environ.get('DOCKER_CONTEXT'))
needs_group = (use_local_socket and os.path.exists(socket_path) and
               not os.access(socket_path, os.R_OK | os.W_OK) and
               os.environ.get('WA1_DOCKER_DIRECT') != '1')
if needs_group:
    try:
        docker_group = grp.getgrnam('docker')
    except KeyError:
        raise SystemExit('Docker socket is inaccessible and the docker group does not exist')
    username = pwd.getpwuid(os.getuid()).pw_name
    if username not in docker_group.gr_mem and os.getgid() != docker_group.gr_gid:
        raise SystemExit(f'{username} cannot access the Docker socket; configure Docker access first')
    if shutil.which('sg') is None:
        raise SystemExit('Docker socket is inaccessible and sg is unavailable')
    os.execvp('sg', ['sg', 'docker', '-c', shlex.join(command)])
os.execv(docker, command)
