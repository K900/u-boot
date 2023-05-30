let
  pkgs = import /home/k900/nixpkgs {
    crossSystem = "aarch64-linux";
  };
  rkbin = pkgs.fetchFromGitHub {
    owner = "radxa";
    repo = "rkbin";
    rev = "d6aad64d4874b416f25669748a9ae5592642a453";
    hash = "sha256-ggPBwvP8dlsR1VWWlHJG2ItP83xVlbrbCrsxcoYW8kw=";
  };
in
  pkgs.buildUBoot {
    src = ./.;
    patches = [];

    version = "unstable-2023-05-30";

    defconfig = "orangepi5-rk3588_defconfig";
    extraMeta.platforms = ["aarch64-linux"];

    BL31 = "${rkbin}/bin/rk35/rk3588_bl31_v1.34.elf";
    ROCKCHIP_TPL = "${rkbin}/bin/rk35/rk3588_ddr_lp4_2112MHz_lp5_2736MHz_v1.08.bin";

    filesToInstall = ["u-boot.itb" "idbloader.img" "spl/u-boot-spl.bin" "u-boot-rockchip-spi.bin" "u-boot-rockchip.bin"];
  }
