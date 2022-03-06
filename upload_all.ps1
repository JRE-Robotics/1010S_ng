# Modes: AWP, BALANCE, RUSH, NONE, SKILLS

Write-Progress -Activity '1010S Auto Uploader' -PercentComplete 0

Read-Host -Prompt "Ensure auto_mode = BALANCE [ENTER]"

Write-Progress -Activity '1010S Auto Uploader' -Status "MAKE: BALANCE auto" -PercentComplete 0
prosv5 make
Write-Progress -Activity '1010S Auto Uploader' -Status "UPLOAD: BALANCE auto" -PercentComplete 0
prosv5 upload --name "BALANCE" --slot 1 --run-screen

Read-Host -Prompt "Ensure auto_mode = AWP [ENTER]"

Write-Progress -Activity '1010S Auto Uploader' -Status "MAKE: AWP auto" -PercentComplete 20
prosv5 make
Write-Progress -Activity '1010S Auto Uploader' -Status "UPLOAD: AWP auto" -PercentComplete 20
prosv5 upload --name "AWP" --slot 2 --run-screen

Read-Host -Prompt "Ensure auto_mode = RUSH [ENTER]"

Write-Progress -Activity '1010S Auto Uploader' -Status "MAKE: RUSH auto" -PercentComplete 40
prosv5 make
Write-Progress -Activity '1010S Auto Uploader' -Status "UPLOAD: RUSH auto" -PercentComplete 40
prosv5 upload --name "RUSH" --slot 3 --run-screen

Read-Host -Prompt "Ensure auto_mode = NONE [ENTER]"

Write-Progress -Activity '1010S Auto Uploader' -Status "MAKE: NONE auto" -PercentComplete 60
prosv5 make
Write-Progress -Activity '1010S Auto Uploader' -Status "UPLOAD: NONE auto" -PercentComplete 60
prosv5 upload --name "NONE" --slot 4 --run-screen

Read-Host -Prompt "Ensure auto_mode = SKILLS [ENTER]"

Write-Progress -Activity '1010S Auto Uploader' -Status "MAKE: SKILLS auto" -PercentComplete 80
prosv5 make
Write-Progress -Activity '1010S Auto Uploader' -Status "UPLOAD: SKILLS auto" -PercentComplete 80
prosv5 upload --name "SKILLS" --slot 5 --run-screen

Write-Progress -Activity '1010S Auto Uploader' -PercentComplete 100
Write-Host "Done!"