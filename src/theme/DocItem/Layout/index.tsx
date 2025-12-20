import React from 'react';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import DocItemLayout from '@theme-original/DocItem/Layout';
import type DocItemLayoutType from '@theme/DocItem/Layout';
import type {WrapperProps} from '@docusaurus/types';

import styles from './styles.module.css';

type Props = WrapperProps<typeof DocItemLayoutType>;

export default function DocItemLayoutWrapper(props: Props): React.ReactElement {
  const {metadata} = useDoc();
  
  return (
    <div className={styles.docItemWrapper}>
      <DocItemLayout {...props} />
    </div>
  );
}
