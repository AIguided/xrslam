import UIKit

@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate {

    var window: UIWindow?

    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        // Programmatic UI: no Main.storyboard in this app.
        let window = UIWindow(frame: UIScreen.main.bounds)
        window.rootViewController = PathRecViewController()
        window.makeKeyAndVisible()
        self.window = window
        return true
    }
}
