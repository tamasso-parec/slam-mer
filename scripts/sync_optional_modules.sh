#!/usr/bin/env bash

set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
repo_name="$(basename "${repo_root}")"
current_branch="$(git -C "${repo_root}" symbolic-ref --quiet --short HEAD || true)"
default_remote=""
remote_url=""
remote_host="github.com"
remote_org=""
remote_style="relative"
super_git_dir=""

if [[ -n "${current_branch}" ]]; then
  default_remote="$(git -C "${repo_root}" config --get "branch.${current_branch}.remote" || true)"
fi

if [[ -z "${default_remote}" ]]; then
  default_remote="origin"
fi

super_git_dir="$(git -C "${repo_root}" rev-parse --git-dir)"
if [[ "${super_git_dir}" != /* ]]; then
  super_git_dir="${repo_root}/${super_git_dir}"
fi

remote_url="$(git -C "${repo_root}" config --get "remote.${default_remote}.url" || true)"
if [[ -n "${remote_url}" ]]; then
  if [[ "${remote_url}" =~ ^https?://([^/:]+)(:[0-9]+)?/([^/]+)/([^/]+?)(\.git)?$ ]]; then
    remote_style="https"
    remote_host="${BASH_REMATCH[1]}"
    remote_org="${BASH_REMATCH[3]}"
    repo_name="${BASH_REMATCH[4]}"
  elif [[ "${remote_url}" =~ ^ssh://git@([^/:]+)(:[0-9]+)?/([^/]+)/([^/]+?)(\.git)?$ ]]; then
    remote_style="ssh-uri"
    remote_host="${BASH_REMATCH[1]}"
    remote_org="${BASH_REMATCH[3]}"
    repo_name="${BASH_REMATCH[4]}"
  elif [[ "${remote_url}" =~ ^git@([^:]+):([^/]+)/([^/]+?)(\.git)?$ ]]; then
    remote_style="ssh-scp"
    remote_host="${BASH_REMATCH[1]}"
    remote_org="${BASH_REMATCH[2]}"
    repo_name="${BASH_REMATCH[3]}"
  else
    repo_name="${remote_url##*/}"
  fi
fi

repo_name="${repo_name%.git}"

suffix=""
if [[ "${repo_name}" == *_private ]]; then
  suffix="_private"
fi

for module_name in depth_inference visual_place_localization; do
  module_path="thirdparty/${module_name}"
  module_key="submodule.${module_path}.url"
  desired_repo="${module_name}${suffix}.git"
  module_git_dir="${super_git_dir}/modules/${module_path}"
  desired_url="../${desired_repo}"

  if [[ -n "${remote_org}" ]]; then
    if [[ "${remote_style}" == "https" ]]; then
      desired_url="https://${remote_host}/${remote_org}/${desired_repo}"
    elif [[ "${remote_style}" == "ssh-uri" ]]; then
      desired_url="ssh://git@${remote_host}/${remote_org}/${desired_repo}"
    else
      desired_url="git@${remote_host}:${remote_org}/${desired_repo}"
    fi
  fi

  # Update local submodule URL config without touching superproject remotes.
  git -C "${repo_root}" config "${module_key}" "${desired_url}"

  # If submodule gitdir exists, keep its origin URL aligned too.
  if [[ -d "${module_git_dir}" ]]; then
    git --git-dir "${module_git_dir}" config remote.origin.url "${desired_url}"
  fi

  if [[ "$(git -C "${repo_root}/${module_path}" rev-parse --show-toplevel 2>/dev/null || true)" == "${repo_root}/${module_path}" ]]; then
    module_remote="$(git -C "${repo_root}/${module_path}" remote | head -n 1 || true)"
    if [[ -n "${module_remote}" ]]; then
      git -C "${repo_root}/${module_path}" remote set-url "${module_remote}" "${desired_url}"
    fi
  fi

  echo "${module_path} -> ${desired_url}"
done
