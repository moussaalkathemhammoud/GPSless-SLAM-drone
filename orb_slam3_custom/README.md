# ORB_SLAM3 customizations

This repo includes `ORB_SLAM3` as a git submodule at `orb_slam3/` (upstream: `UZ-SLAMLab/ORB_SLAM3`), plus local changes captured from the working copy in `orb_slam3_custom/`.

## Apply local changes

```bash
git submodule update --init --recursive

# Apply tracked-file edits
git -C orb_slam3 apply ../orb_slam3_custom/patches/orb_slam3_local_changes.patch

# Copy additional untracked helper files (bridge/scripts/configs)
rsync -a orb_slam3_custom/files/ orb_slam3/
```

