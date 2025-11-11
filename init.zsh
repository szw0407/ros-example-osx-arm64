#!/usr/bin/env zsh

echo "[NIX] Setting up Conda environment from ./env.yaml ..."
export MAMBA_ROOT_PREFIX="$PWD/.conda"
# 读取 env.yaml 里的 name 字段
ENV_NAME=$(awk '/^name:/ {print $2}' ./env.yaml)
export CONDA_PREFIX="$MAMBA_ROOT_PREFIX/envs/$ENV_NAME"

if [ ! -d "$CONDA_PREFIX" ]; then
  echo "[NIX] Creating conda environment $ENV_NAME in ./.conda ..."
  if type micromamba &>/dev/null; then
    micromamba create -y -f ./env.yaml -p "$CONDA_PREFIX"
  elif type mamba &>/dev/null; then
    mamba env create -f ./env.yaml -p "$CONDA_PREFIX"
  elif type conda &>/dev/null; then
    conda env create -f ./env.yaml -p "$CONDA_PREFIX"
  else
    echo "[ERROR] micromamba/mamba/conda can't be found, cannot create environment. For nix users, use zsh shell and run 'nix develop' to enter the dev shell."
    return 1
  fi
fi

# 激活 micromamba 环境
# shell hook 只需执行一次
if type micromamba &>/dev/null; then
  eval "$(micromamba shell hook --shell zsh)"
  micromamba activate "$CONDA_PREFIX"
elif type mamba &>/dev/null; then
  source "$(dirname $(which mamba))/../etc/profile.d/conda.sh"
  conda activate "$CONDA_PREFIX"
elif type conda &>/dev/null; then
  source "$(dirname $(which conda))/../etc/profile.d/conda.sh"
  conda activate "$CONDA_PREFIX"
else
  echo "[ERROR] micromamba/mamba/conda can't be found, cannot activate environment. For nix users, use zsh shell and run 'nix develop' to enter the dev shell."
  return 1
fi

# 自动 source devel/setup.zsh（如果存在）
if [ -f "$PWD/devel/setup.zsh" ]; then
  source "$PWD/devel/setup.zsh"
fi

echo "[ROS1] Environment ready!"
echo "Run: roscore  / rostopic list  / rosrun turtlesim turtlesim_node"
