# Connect to MiRco

## Connect to `MiRco PC`
`MiRco PC` is connected to the lab network however it does not have a static `MiRco_IP` (for now).

### If you know `MiRco_IP`
If you know `MiRco_IP` you can connect to `MiRco PC` in the following ways:

- **Ssh**: `ssh student@MiRco_IP` and sign in.
- **Remote desktop**: To use remote desktop `MiRco PC` needs a connected monitor so **make sure the virtual DisplayPort adapter is plugged in**.  
Use AnyDesk, `MiRco PC` shows up under network discovered devices. If not, use `MiRco_IP`.
- **Connect a display, computer and mouse directly to the computer** 

### If you don't know `MiRco_IP`
Connect to the computer over remote desktop (AnyDesk). You have to be connected to the same network (lab network by default). If that doesn't work plug in a display. When connected, open the terminal and look up `MiRco_IP` with `ifconfig`.

## Development tips
- Connection over remote desktop will always be lacking. Use it only when visual tools like `RViz` are needed. When possible connect to the robot over `ssh`. 
- When writing or testing code on `MiRco`, use the `Remote-SSH` extension for `VSCodium` or `VSCode`. It enables you to remotely access files on the MiRco computer within your development environment. There is no lag issue like with remote desktop and the robot is still free to move (as opposed to connecting a monitor).
- If you are using a Docker installation on your robot, use the `Dev Containers` extension for `VSCode`. It enables you to access container files within your development environment. If you use it in combination with the `Remote-SSH` extension, you can access files within a container running on your robot computer. This extension is only available for `VSCode` and not for `VSCodium`. 