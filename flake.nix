{
  description = "Nix + Conda (RoboStack ROS1 Noetic) example";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-25.05-darwin";
    flake-utils.url = "github:numtide/flake-utils";
  };
  
  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        # micromamba 用于轻量级 conda 管理
        micromamba = pkgs.micromamba;

        # 使用本地 env.yaml 作为 Conda 环境定义
        condaEnv = ./env.yaml;
      in {
        devShells.default = pkgs.mkShell {
          name = "ros1-conda";
          packages = [ micromamba pkgs.bashInteractive ];

          shellHook = ''
            source "$PWD/init.zsh"
          '';
        };
      });
}

