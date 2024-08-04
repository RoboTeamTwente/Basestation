import subprocess


def is_container_running(image_name):
	"""Check if a Docker container with the specified image name is running."""
	result = subprocess.run(['docker', 'ps', '--filter', f"ancestor={image_name}", '--format', '{{.Image}}'], capture_output=True, text=True)
	return image_name in result.stdout

def kill_docker_containers_by_image(image_name):
	"""Kill all running Docker containers with the specified image name."""
	try:
		# List all running containers
		cmd_list_containers = ["docker", "ps", "-q", "--filter", f"ancestor={image_name}"]
		running_containers = subprocess.check_output(cmd_list_containers).decode('utf-8').strip().split('\n')

		# Check if there's any container to kill
		if running_containers == ['']:
			print("[SerialSimulator] No running containers found for the specified image.")
			return

		# Kill each container found
		for container_id in running_containers:
			subprocess.check_call(["docker", "kill", container_id])
			print(f"[SerialSimulator] Container {container_id} killed.")
	except subprocess.CalledProcessError as e:
		print(f"[SerialSimulator] Error killing containers: {e}")