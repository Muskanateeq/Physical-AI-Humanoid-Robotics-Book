import React from 'react';
import { useSession, signOut } from '@site/src/lib/auth-client';
import { useHistory } from '@docusaurus/router';
import styles from './styles.module.css';

export default function NavbarAuth() {
  const { data: session, isPending } = useSession();
  const history = useHistory();

  const handleSignOut = async () => {
    await signOut();
    history.push('/');
  };

  if (isPending) {
    return null;
  }

  if (session?.user) {
    return (
      <div className={styles.userMenu}>
        <span className={styles.userName}>
          {session.user.name || session.user.email}
        </span>
        <button onClick={handleSignOut} className={styles.signOutBtn}>
          Sign Out
        </button>
      </div>
    );
  }

  return (
    <div className={styles.authButtons}>
      <a href="/signin" className={styles.signInBtn}>
        Sign In
      </a>
      <a href="/signup" className={styles.signUpBtn}>
        Sign Up
      </a>
    </div>
  );
}
