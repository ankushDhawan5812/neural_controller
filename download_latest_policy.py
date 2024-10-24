import wandb
import pathlib
import shutil


def download_latest_model(
    project_name, entity_name, model_dir="launch", model_name="policy_latest.json"
):
    """
    Downloads the latest model from a W&B project.

    :param project_name: The name of the W&B project.
    :param entity_name: The W&B entity (username or team).
    :param model_dir: The directory where the model will be downloaded.
    :param model_name: The name to copy the model as.
    :return: None
    """

    # Initialize the API
    api = wandb.Api()

    # Fetch the latest run
    runs = api.runs(f"{entity_name}/{project_name}")

    # Check if there are any runs
    if not runs:
        print("No runs found in the project.")
        return

    # Get the latest run (assuming runs are sorted by start time by default)
    # sort runs by the number at the end of the name
    runs = sorted(runs, key=lambda run: int(run.name.split("-")[-1]))
    latest_run = runs[-1]

    print(f"Latest run: {latest_run.name}")

    # get the artifact with the name that contains .json
    art = [art for art in latest_run.logged_artifacts() if ".json" in art.name][0]
    print("Using: ", art.name)

    # remove the :[version] from the name
    base_name = art.name.split(":")[0]
    print("Base name: ", base_name)

    # get folder of this script
    script_dir = pathlib.Path(__file__).parent
    model_dir = pathlib.Path(model_dir)

    downloaded_filepath = script_dir / model_dir / pathlib.Path(base_name)
    save_filepath = script_dir / model_dir / model_name

    # Download the file
    art.download(root=script_dir / model_dir)
    print(f"Model downloaded to: {downloaded_filepath}")
    model_name = pathlib.Path(model_name)
    shutil.copyfile(downloaded_filepath, save_filepath)
    print(f"Model copied to: {save_filepath}")


if __name__ == "__main__":
    # Define your project and entity (username or team)
    project_name = "pupperv3-mjx-rl"
    entity_name = "hands-on-robotics"

    # Call the function to download the latest model
    download_latest_model(
        project_name, entity_name, model_dir="launch", model_name="policy_latest.json"
    )
