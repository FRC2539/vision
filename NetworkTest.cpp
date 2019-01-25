#include <networktables/NetworkTable.h>
#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

int main()
{
	NetworkTable::SetTeam(2539);
	NetworkTable::SetIPAddress("roborio-2539-frc.local");
	NetworkTable::SetClientMode();
	std::shared_ptr<NetworkTable> targetInfo =
		NetworkTable::GetTable("SmartDashboard");
	std::cout << "Connected to SmartDashboard table\n";
    std::this_thread::sleep_for (std::chrono::seconds(1));
	targetInfo->PutBoolean("Monkey", true);
	std::cout << "Set 'Working' to 'true'\n";
    std::this_thread::sleep_for (std::chrono::seconds(1));
	std::cout << targetInfo->GetString("Hello", "FAIL") << "\n";
    std::this_thread::sleep_for (std::chrono::seconds(1));
	NetworkTable::Shutdown();
}