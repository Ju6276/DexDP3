# use the same command as training except the script
# for example:
# bash scripts/deploy_policy.sh simple_dp3 realdex_dumpling 0112 0 0
# bash scripts/deploy_policy.sh dp3 realdex_push 2013 0 0
# bash scripts/deploy_policy.sh simple_dp3 realdex_push 2248 0 0
# bash scripts/deploy_policy.sh dp3 realdex_grasp 1314 0 0

DEBUG=False
wandb_mode="offline"
save_ckpt=False

alg_name=${1}
task_name=${2}
config_name=${alg_name}
addition_info=${3}
seed=${4}
exp_name=${task_name}-${alg_name}-${addition_info}
run_dir="/home/ju/3D-Diffusion-Policy/3D-Diffusion-Policy/data/outputs/${exp_name}_seed${seed}"

gpu_id=${5}

cd 3D-Diffusion-Policy

export HYDRA_FULL_ERROR=1
export CUDA_VISIBLE_DEVICES=${gpu_id}
python deploy.py --config-name=${config_name}.yaml \
                            task=${task_name} \
                            +run_dir=${run_dir} \
                            training.debug=$DEBUG \
                            training.seed=${seed} \
                            training.device="cuda:0" \
                            exp_name=${exp_name} \
                            logging.mode=${wandb_mode} \
                            checkpoint.save_ckpt=${save_ckpt}


                                