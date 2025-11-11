#!/usr/bin/env zsh

echo "[NIX] Setting up Conda environment from ./env.yaml ..."

# 检查 conda/mamba/micromamba 是否已安装
if ! type micromamba &>/dev/null && ! type mamba &>/dev/null && ! type conda &>/dev/null; then
  echo "[ERROR] micromamba/mamba/conda not found. Run \`nix develop\` to enter the dev shell."
  return 1
fi

# 读取 env.yaml 里的 name 字段
ENV_NAME=$(awk '/^name:/ {print $2}' ./env.yaml)

# 检查该环境是否已存在
# ENV_EXISTS=0
if type conda &>/dev/null; then
  if conda env list | grep -q "^${ENV_NAME}"; then
    # ENV_EXISTS=1
    conda activate $ENV_NAME
  fi
elif type mamba &>/dev/null; then
  if mamba env list | grep -q "^${ENV_NAME}"; then
    # ENV_EXISTS=1
    mamba activate $ENV_NAME
  fi
elif type micromamba &>/dev/null; then
  if micromamba env list | grep -q "^${ENV_NAME}"; then
    # ENV_EXISTS=1
    micromamba activate $ENV_NAME
  fi
else
  export MAMBA_ROOT_PREFIX="$PWD/.conda"
  export CONDA_PREFIX="$MAMBA_ROOT_PREFIX/envs/$ENV_NAME"
  echo "[NIX] env named '$ENV_NAME' will be created at $CONDA_PREFIX"
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

  # 激活 conda/mamba/micromamba 环境
  # shell hook 只需执行一次，默认用户 shell 已经执行过了，这只用于处理 nix develop 进入的 shell而带来的micromamba未hook问题
  if type micromamba &>/dev/null; then
    eval "$(micromamba shell hook --shell zsh)"
    micromamba activate "$CONDA_PREFIX"
  fi
fi

# 自动 source devel/setup.zsh（如果存在）
if [ -f "$PWD/devel/setup.zsh" ]; then
  source "$PWD/devel/setup.zsh"
fi

echo "[ROS1] Environment ready!"
echo "Run: roscore  / rostopic list  / rosrun turtlesim turtlesim_node"
