import React from "react";
import { useSession, signOut } from "../../lib/auth-client";
import { useHistory } from "@docusaurus/router";
import styles from "./AuthNavbar.module.css";

export default function AuthNavbar() {
  const { data: session, isPending } = useSession();
  const history = useHistory();

  const handleSignOut = async () => {
    await signOut();
    history.push("/");
  };

  if (isPending) {
    return null; // Don't show anything while loading
  }

  return (
    <div className={styles.authSection}>
      {session?.user ? (
        // User is logged in
        <div className={styles.userMenu}>
          <div className={styles.userInfo}>
            {session.user.image ? (
              <img
                src={session.user.image}
                alt={session.user.name || "User"}
                className={styles.avatar}
              />
            ) : (
              <div className={styles.avatarPlaceholder}>
                {(session.user.name || session.user.email || "U")[0].toUpperCase()}
              </div>
            )}
            <span className={styles.userName}>
              {session.user.name || session.user.email}
            </span>
          </div>
          <button onClick={handleSignOut} className={styles.signOutButton}>
            Sign Out
          </button>
        </div>
      ) : (
        // User is not logged in
        <div className={styles.authButtons}>
          <a href="/signin" className={styles.signInButton}>
            Sign In
          </a>
          <a href="/signup" className={styles.signUpButton}>
            Sign Up
          </a>
        </div>
      )}
    </div>
  );
}
